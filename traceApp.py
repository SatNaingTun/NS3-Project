import matplotlib.pyplot as plt
import pandas as pd
from tkinter import Tk, filedialog, Frame, BOTH, Button, Canvas, Label, LEFT, RIGHT, TOP, BOTTOM, X
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# === Global containers ===
df = pd.DataFrame()
stats = {}
fig = None
ax = None
summary_ax = None
summary_table_data = []

# === Main Window ===
window = Tk()
window.title("NS-3 Trace Viewer")
window.geometry("1400x800")

def on_close():
    print("Window closed by user.")
    window.destroy()

window.protocol("WM_DELETE_WINDOW", on_close)

# === Top Toolbar ===
toolbar_frame = Frame(window)
toolbar_frame.pack(side=TOP, fill=X, pady=5)

def load_trace_file():
    global df, stats, fig, ax, summary_ax, summary_table_data

    file_path = filedialog.askopenfilename(title="Select .tr file", filetypes=[("Trace Files", "*.tr")])
    if not file_path:
        return

    packet_data = []
    with open(file_path) as f:
        for idx, line in enumerate(f):
            if line.startswith('r'):
                parts = line.strip().split()
                timestamp = float(parts[1])
                src = next((p for p in parts if p.startswith("from")), "Unknown")
                dst = parts[2].rstrip(':') if "Node" in parts[2] else "Unknown"
                size = next((p.split('=')[1] for p in parts if p.startswith("size=")), "N/A")
                proto = next((p.split('=')[1] for p in parts if p.startswith("protocol=")), "N/A")
                packet_data.append({
                    "Packet Index": idx,
                    "Reception Time (s)": timestamp,
                    "Source Node": src,
                    "Destination Node": dst,
                    "Size (bytes)": size,
                    "Protocol": proto
                })

    df = pd.DataFrame(packet_data)
    stats = {
        'Total Packets Received': len(df),
        'First Packet Time (s)': round(df['Reception Time (s)'].min(), 6),
        'Last Packet Time (s)': round(df['Reception Time (s)'].max(), 6),
        'Duration (s)': round(df['Reception Time (s)'].max() - df['Reception Time (s)'].min(), 6),
        'Average Interval (s)': round(df['Reception Time (s)'].diff().mean(), 6)
    }

    # === Update Summary Table ===
    summary_canvas.delete("all")
    summary_ax = plt.Figure(figsize=(4, 2)).add_subplot(111)
    summary_ax.axis('off')
    summary_table_data = [[k, v] for k, v in stats.items()]
    table = summary_ax.table(cellText=summary_table_data, colLabels=["Metric", "Value"], loc='center')
    table.scale(1, 2)
    summary_plot = FigureCanvasTkAgg(summary_ax.figure, master=summary_canvas)
    summary_plot.draw()
    summary_plot.get_tk_widget().pack()

    # === Update Plot ===
    ax.clear()
    ax.plot(df['Reception Time (s)'], marker='o', linestyle='-', color='blue')
    ax.set_title("Packet Reception Over Time", fontweight='bold')
    ax.set_xlabel("Packet Index")
    ax.set_ylabel("Time (s)")
    ax.grid(True)
    plot_canvas.draw()

def save_summary():
    path = filedialog.asksaveasfilename(
        title="Save Summary",
        filetypes=[
            ("CSV File", "*.csv"),
            ("PNG Image", "*.png"),
            ("JPEG Image", "*.jpg"),
            ("PDF Document", "*.pdf"),
        ]
    )
    if not path:
        return

    ext = path.split('.')[-1].lower()
    if ext == "csv":
        pd.DataFrame(summary_table_data, columns=["Metric", "Value"]).to_csv(path, index=False)
        print(f"Summary saved to CSV: {path}")
    elif ext in ["png", "jpg","pdf"]:
        fig_summary = plt.Figure(figsize=(4, 2))
        ax_summary = fig_summary.add_subplot(111)
        ax_summary.axis('off')
        table = ax_summary.table(cellText=summary_table_data, colLabels=["Metric", "Value"], loc='center')
        table.scale(1, 2)
        fig_summary.savefig(path)
        print(f"Summary saved as image: {path}")
    else:
        print("Unsupported format. Please use CSV, PNG, or JPG.")


def save_detail_csv():
    if df.empty:
        return
    path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV File", "*.csv")])
    if path:
        df.to_csv(path, index=False)
        print(f"Detail saved to: {path}")

def save_plot():
    path = filedialog.asksaveasfilename(
        title="Save Plot",
        filetypes=[
            ("PNG Image", "*.png"),
            ("JPEG Image", "*.jpg"),
            ("PDF Document", "*.pdf"),
            ("SVG Vector", "*.svg"),
            ("EPS File", "*.eps")
        ]
    )
    if path:
        fig.savefig(path)
        print(f"Plot saved to: {path}")

def save_combined():
    path = filedialog.asksaveasfilename(
        title="Save Combined Summary and Plot",
        filetypes=[
            ("PNG Image", "*.png"),
            ("JPEG Image", "*.jpg"),
            ("PDF Document", "*.pdf")
        ]
    )
    if not path:
        return

    ext = path.split('.')[-1].lower()
    if ext not in ["png", "jpg", "pdf"]:
        print("Unsupported format. Please use PNG, JPG, or PDF.")
        return

    # Create combined figure
    combined_fig = plt.figure(figsize=(10, 8))

    # Plot section
    plot_ax = combined_fig.add_subplot(211)
    plot_ax.plot(df['Reception Time (s)'], marker='o', linestyle='-', color='blue')
    plot_ax.set_title("Packet Reception Over Time", fontweight='bold')
    plot_ax.set_xlabel("Packet Index")
    plot_ax.set_ylabel("Time (s)")
    plot_ax.grid(True)

    # Summary table section
    table_ax = combined_fig.add_subplot(212)
    table_ax.axis('off')
    table = table_ax.table(cellText=summary_table_data, colLabels=["Metric", "Value"], loc='center')
    table.scale(1, 2)
    table_ax.set_title("Trace Summary", fontweight='bold')

    combined_fig.tight_layout()
    combined_fig.savefig(path)
    print(f"Combined image saved to: {path}")

# === Toolbar Buttons ===
Button(toolbar_frame, text="Open File", command=load_trace_file).pack(side=LEFT, padx=10)
Button(toolbar_frame, text="Save Summary Table", command=save_summary).pack(side=LEFT, padx=10)
Button(toolbar_frame, text="Save Detail as CSV", command=save_detail_csv).pack(side=LEFT, padx=10)
Button(toolbar_frame, text="Save Plot", command=save_plot).pack(side=LEFT, padx=10)
Button(toolbar_frame, text="Save Combined", command=save_combined).pack(side=LEFT, padx=10)

# === Main Split Frame ===
main_frame = Frame(window)
main_frame.pack(fill=BOTH, expand=True)

left_panel = Frame(main_frame)
left_panel.pack(side=LEFT, fill=BOTH, expand=True)

right_panel = Frame(main_frame)
right_panel.pack(side=RIGHT, fill=BOTH, expand=True)

# === Summary Table ===
summary_frame = Frame(left_panel)
summary_frame.pack(side=TOP, fill=BOTH, expand=False, pady=10)

Label(summary_frame, text="Trace Summary", font=("Arial", 14, "bold")).pack(side=TOP, pady=5)

summary_canvas = Canvas(summary_frame)
summary_canvas.pack(side=TOP, fill=BOTH, expand=True)

# === Plot Panel ===
plot_frame = Frame(right_panel)
plot_frame.pack(fill=BOTH, expand=True, pady=10)

fig = plt.Figure(figsize=(8, 6))
ax = fig.add_subplot(111)
plot_canvas = FigureCanvasTkAgg(fig, master=plot_frame)
plot_canvas.get_tk_widget().pack(fill=BOTH, expand=True)

toolbar = NavigationToolbar2Tk(plot_canvas, plot_frame)
toolbar.update()
toolbar.pack(side=BOTTOM, fill=X)

# === Launch Window and Auto File Dialog ===
window.after(100, load_trace_file)
window.mainloop()