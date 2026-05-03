#!/bin/bash
# =============================================================================
# Mac Developer Setup Script
# Installs dev tools, apps, and configs on a fresh macOS machine.
# Usage: bash scripts/setup_mac.sh
# =============================================================================

set -e

# =============================================================================
echo "==> [1/7] Installing Homebrew..."
# =============================================================================
if ! command -v brew &>/dev/null; then
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    # Add brew to PATH for Apple Silicon
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
    eval "$(/opt/homebrew/bin/brew shellenv)"
else
    echo "Homebrew already installed — updating..."
    brew update
fi

# =============================================================================
echo "==> [2/7] Installing CLI tools..."
# =============================================================================
brew install \
    tmux \
    neovim \
    git \
    gh \
    fzf \
    ripgrep \
    fd \
    bat \
    eza \
    zoxide \
    starship \
    node \
    python@3.12 \
    wget \
    jq \
    lazygit

# fzf shell integration
$(brew --prefix)/opt/fzf/install --all --no-bash --no-fish

# =============================================================================
echo "==> [3/7] Installing GUI apps..."
# =============================================================================
brew install --cask \
    iterm2 \
    discord \
    foxglove-studio \
    visual-studio-code \
    cursor \
    obsidian \
    raycast

# =============================================================================
echo "==> [4/7] Installing Claude Code..."
# =============================================================================
npm install -g @anthropic-ai/claude-code

# =============================================================================
echo "==> [5/7] Setting up LazyVim..."
# =============================================================================
# Backup existing nvim config if present
[ -d ~/.config/nvim ] && mv ~/.config/nvim ~/.config/nvim.bak.$(date +%s)
[ -d ~/.local/share/nvim ] && mv ~/.local/share/nvim ~/.local/share/nvim.bak.$(date +%s)

git clone https://github.com/LazyVim/starter ~/.config/nvim
rm -rf ~/.config/nvim/.git
mkdir -p ~/.config/nvim/lua/plugins

# lazygit integration
cat > ~/.config/nvim/lua/plugins/lazygit.lua << 'LUACONF'
return {
  {
    "folke/snacks.nvim",
    opts = {
      lazygit = { enabled = true },
    },
    keys = {
      { "<leader>gg", function() Snacks.lazygit() end,          desc = "Lazygit" },
      { "<leader>gG", function() Snacks.lazygit.log() end,      desc = "Lazygit log" },
      { "<leader>gf", function() Snacks.lazygit.log_file() end, desc = "Lazygit file log" },
    },
  },
}
LUACONF

# diffview: side-by-side diff against any branch
cat > ~/.config/nvim/lua/plugins/diffview.lua << 'LUACONF'
return {
  "sindrets/diffview.nvim",
  dependencies = { "nvim-lua/plenary.nvim" },
  cmd = { "DiffviewOpen", "DiffviewFileHistory" },
  keys = {
    { "<leader>gd", function() vim.cmd("DiffviewOpen main -- " .. vim.fn.expand("%")) end, desc = "Diff file vs main" },
    { "<leader>gD", function()
        local b = vim.fn.input("Branch: ", "main")
        if b ~= "" then vim.cmd("DiffviewOpen " .. b .. " -- " .. vim.fn.expand("%")) end
      end, desc = "Diff file vs branch" },
    { "<leader>gm", "<cmd>DiffviewOpen main<cr>",     desc = "Diff repo vs main" },
    { "<leader>gh", "<cmd>DiffviewFileHistory %<cr>", desc = "File history" },
    { "<leader>gq", "<cmd>DiffviewClose<cr>",         desc = "Close diffview" },
  },
  opts = {
    view = {
      default      = { layout = "diff2_vertical" },
      file_history = { layout = "diff2_vertical" },
    },
  },
}
LUACONF

echo "LazyVim installed with lazygit + diffview plugins"

# =============================================================================
echo "==> [6/7] Setting up tmux (TPM + sensible config)..."
# =============================================================================
# Install TPM
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm 2>/dev/null || \
    git -C ~/.tmux/plugins/tpm pull

# Write tmux config if none exists
if [ ! -f ~/.tmux.conf ]; then
cat > ~/.tmux.conf << 'TMUXCONF'
# =============================================================================
# tmux config
# =============================================================================

# Change prefix to Ctrl+a
unbind C-b
set -g prefix C-a
bind C-a send-prefix

# Mouse support
set -g mouse on

# Start windows and panes at 1
set -g base-index 1
setw -g pane-base-index 1

# Renumber windows on close
set -g renumber-windows on

# Increase history
set -g history-limit 50000

# Fast escape (for nvim)
set -sg escape-time 10

# True color
set -g default-terminal "tmux-256color"
set -ag terminal-overrides ",xterm-256color:RGB"

# Split panes with | and -
bind | split-window -h -c "#{pane_current_path}"
bind - split-window -v -c "#{pane_current_path}"

# Pane navigation with vim keys
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R

# Reload config
bind r source-file ~/.tmux.conf \; display "Config reloaded"

# Plugins
set -g @plugin 'tmux-plugins/tpm'
set -g @plugin 'tmux-plugins/tmux-sensible'
set -g @plugin 'catppuccin/tmux'
set -g @plugin 'tmux-plugins/tmux-yank'

set -g @catppuccin_flavour 'mocha'

run '~/.tmux/plugins/tpm/tpm'
TMUXCONF
    echo "tmux config written to ~/.tmux.conf"
else
    echo "~/.tmux.conf already exists — skipping"
fi

# Install tmux plugins
~/.tmux/plugins/tpm/bin/install_plugins

# =============================================================================
echo "==> [7/7] Configuring shell (zsh)..."
# =============================================================================
ZSHRC=~/.zshrc
add_to_zshrc() {
    grep -qxF "$1" $ZSHRC 2>/dev/null || echo "$1" >> $ZSHRC
}

# Starship prompt
add_to_zshrc 'eval "$(starship init zsh)"'

# zoxide (smarter cd)
add_to_zshrc 'eval "$(zoxide init zsh)"'

# eza aliases (better ls)
add_to_zshrc 'alias ls="eza --icons"'
add_to_zshrc 'alias ll="eza -la --icons --git"'
add_to_zshrc 'alias lt="eza --tree --icons -L 2"'

# bat alias (better cat)
add_to_zshrc 'alias cat="bat"'

# lazygit alias
add_to_zshrc 'alias lg="lazygit"'

# =============================================================================
echo ""
echo "============================================================"
echo " Mac setup complete!"
echo "============================================================"
echo ""
echo " Next steps:"
echo "   1. Restart terminal or: source ~/.zshrc"
echo "   2. Open nvim to finish LazyVim plugin install"
echo "   3. Start tmux: tmux new -s main"
echo "   4. Claude Code: claude"
echo "   5. iTerm2: Enable tmux integration in Preferences → General → tmux"
echo "============================================================"
