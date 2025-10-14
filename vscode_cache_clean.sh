#!/bin/bash

echo "🧹 Cleaning VS Code cache..."

# Remove C++ extension cache
rm -rf ~/.cache/vscode-cpptools

# Optional: Remove other safe-to-delete cache folders
rm -rf ~/.config/Code/CachedData
rm -rf ~/.config/Code/User/workspaceStorage
rm -rf ~/.config/Code/User/globalStorage

echo "✅ VS Code cache cleaned. Restart VS Code to rebuild fresh cache."