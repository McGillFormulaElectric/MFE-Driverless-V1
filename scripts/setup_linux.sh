#!/bin/bash
# =============================================================================
# Linux Developer Setup Script (Ubuntu 22.04 / 24.04 / 26.04)
# Installs dev tools, apps, and configs.
# Usage: bash scripts/setup_linux.sh
# =============================================================================

set -e

ARCH=$(dpkg --print-architecture)
UBUNTU_VERSION=$(lsb_release -rs)

echo "==> Ubuntu $UBUNTU_VERSION ($ARCH)"

# =============================================================================
echo "==> [1/8] System packages..."
# =============================================================================
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    curl wget git unzip tar \
    build-essential cmake \
    tmux \
    ripgrep fd-find fzf bat \
    zsh \
    python3-pip python3-venv \
    xclip xsel \
    fontconfig

# JetBrains Mono Nerd Font (used by WezTerm + nvim icons)
FONT_DIR=~/.local/share/fonts/JetBrainsMono
mkdir -p "$FONT_DIR"
NERD_VERSION=$(curl -s https://api.github.com/repos/ryanoasis/nerd-fonts/releases/latest \
    | grep '"tag_name"' | cut -d'"' -f4)
curl -Lo /tmp/JetBrainsMono.zip \
    "https://github.com/ryanoasis/nerd-fonts/releases/download/${NERD_VERSION}/JetBrainsMono.zip"
unzip -o /tmp/JetBrainsMono.zip "*.ttf" -d "$FONT_DIR"
rm /tmp/JetBrainsMono.zip
fc-cache -f
echo "JetBrains Mono Nerd Font installed"

# fd is called fdfind on Ubuntu, symlink it
sudo ln -sf $(which fdfind) /usr/local/bin/fd 2>/dev/null || true
# bat is called batcat on Ubuntu, symlink it
sudo ln -sf $(which batcat) /usr/local/bin/bat 2>/dev/null || true

# =============================================================================
echo "==> [2/8] Installing Neovim (latest)..."
# =============================================================================
# Ubuntu apt ships an old nvim — install latest from GitHub
NVIM_VERSION=$(curl -s https://api.github.com/repos/neovim/neovim/releases/latest \
    | grep '"tag_name"' | cut -d'"' -f4)
curl -LO "https://github.com/neovim/neovim/releases/download/${NVIM_VERSION}/nvim-linux-${ARCH}.tar.gz"
sudo rm -rf /opt/nvim
sudo tar -C /opt -xzf nvim-linux-${ARCH}.tar.gz
sudo ln -sf /opt/nvim-linux-${ARCH}/bin/nvim /usr/local/bin/nvim
rm nvim-linux-${ARCH}.tar.gz
echo "Neovim ${NVIM_VERSION} installed"

# =============================================================================
echo "==> [3/8] Installing LazyVim..."
# =============================================================================
[ -d ~/.config/nvim ] && mv ~/.config/nvim ~/.config/nvim.bak.$(date +%s)
[ -d ~/.local/share/nvim ] && mv ~/.local/share/nvim ~/.local/share/nvim.bak.$(date +%s)
git clone https://github.com/LazyVim/starter ~/.config/nvim
rm -rf ~/.config/nvim/.git

# Add lazygit plugin config
mkdir -p ~/.config/nvim/lua/plugins
cat > ~/.config/nvim/lua/plugins/lazygit.lua << 'LUACONF'
return {
  -- lazygit integration via snacks.nvim (bundled with LazyVim)
  {
    "folke/snacks.nvim",
    opts = {
      lazygit = {
        -- use the system lazygit binary installed by setup_linux.sh
        enabled = true,
      },
    },
    keys = {
      { "<leader>gg", function() Snacks.lazygit() end,        desc = "Lazygit" },
      { "<leader>gG", function() Snacks.lazygit.log() end,    desc = "Lazygit log (cwd)" },
      { "<leader>gf", function() Snacks.lazygit.log_file() end, desc = "Lazygit current file" },
    },
  },
}
LUACONF

echo "LazyVim installed with lazygit keybinds: <leader>gg / gG / gf"

# diffview: side-by-side file diff against any branch
cat > ~/.config/nvim/lua/plugins/diffview.lua << 'LUACONF'
return {
  "sindrets/diffview.nvim",
  dependencies = { "nvim-lua/plenary.nvim" },
  cmd = { "DiffviewOpen", "DiffviewFileHistory" },
  keys = {
    -- Diff current file against main (vertical split)
    {
      "<leader>gd",
      function()
        local file = vim.fn.expand("%")
        vim.cmd("DiffviewOpen main -- " .. file)
      end,
      desc = "Diff file vs main",
    },
    -- Diff current file against a branch you type
    {
      "<leader>gD",
      function()
        local branch = vim.fn.input("Diff against branch: ", "main")
        if branch ~= "" then
          local file = vim.fn.expand("%")
          vim.cmd("DiffviewOpen " .. branch .. " -- " .. file)
        end
      end,
      desc = "Diff file vs branch...",
    },
    -- Full repo diff against main
    { "<leader>gm", "<cmd>DiffviewOpen main<cr>",           desc = "Diff repo vs main" },
    -- File history (log with diffs) for current file
    { "<leader>gh", "<cmd>DiffviewFileHistory %<cr>",        desc = "File git history" },
    -- Close diffview
    { "<leader>gq", "<cmd>DiffviewClose<cr>",                desc = "Close diffview" },
  },
  opts = {
    view = {
      default = { layout = "diff2_vertical" },   -- always vertical split
      file_history = { layout = "diff2_vertical" },
    },
  },
}
LUACONF

# =============================================================================
echo "==> [4/8] Installing Node.js (for Claude Code)..."
# =============================================================================
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install -y nodejs
echo "Node $(node --version) installed"

# =============================================================================
echo "==> [5/8] Installing Claude Code..."
# =============================================================================
sudo npm install -g @anthropic-ai/claude-code
echo "Claude Code installed: $(claude --version 2>/dev/null || echo 'restart shell to verify')"

# =============================================================================
echo "==> [5b] Installing GitHub CLI + gh-dash..."
# =============================================================================
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | sudo tee /etc/apt/sources.list.d/github-cli.list
sudo apt update && sudo apt install -y gh
gh extension install dlvhdr/gh-dash
echo "gh-dash installed — run: gh dash"

# =============================================================================
echo "==> [6/8] Installing eza, zoxide, starship..."
# =============================================================================
# eza (better ls)
sudo mkdir -p /etc/apt/keyrings
wget -qO- https://raw.githubusercontent.com/eza-community/eza/main/deb.asc \
    | sudo gpg --dearmor -o /etc/apt/keyrings/gierens.gpg
echo "deb [signed-by=/etc/apt/keyrings/gierens.gpg] http://deb.gierens.de stable main" \
    | sudo tee /etc/apt/sources.list.d/gierens.list
sudo chmod 644 /etc/apt/keyrings/gierens.gpg /etc/apt/sources.list.d/gierens.list
sudo apt update && sudo apt install -y eza

# zoxide (smarter cd)
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh

# starship prompt
curl -sS https://starship.rs/install.sh | sh -s -- --yes

# lazygit
LAZYGIT_VERSION=$(curl -s https://api.github.com/repos/jesseduffield/lazygit/releases/latest \
    | grep '"tag_name"' | cut -d'"' -f4 | sed 's/v//')
curl -Lo lazygit.tar.gz \
    "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_${ARCH}.tar.gz"
tar xf lazygit.tar.gz lazygit
sudo install lazygit -D -t /usr/local/bin/
rm lazygit lazygit.tar.gz

# =============================================================================
echo "==> [6b] Installing WezTerm..."
# =============================================================================
curl -fsSL https://apt.fury.io/wez/gpg.key \
    | sudo gpg --yes --dearmor -o /usr/share/keyrings/wezterm-fury.gpg
echo "deb [signed-by=/usr/share/keyrings/wezterm-fury.gpg] https://apt.fury.io/wez/ * *" \
    | sudo tee /etc/apt/sources.list.d/wezterm.list
sudo apt update && sudo apt install -y wezterm

# Write a base WezTerm config
mkdir -p ~/.config/wezterm
cat > ~/.config/wezterm/wezterm.lua << 'WEZCONF'
local wezterm = require("wezterm")
local config = wezterm.config_builder()

-- Font
config.font = wezterm.font("JetBrains Mono", { weight = "Regular" })
config.font_size = 13.0

-- Theme
config.color_scheme = "Catppuccin Mocha"

-- Window
config.window_background_opacity = 0.95
config.window_padding = { left = 8, right = 8, top = 8, bottom = 8 }
config.enable_tab_bar = true
config.hide_tab_bar_if_only_one_tab = true
config.window_decorations = "RESIZE"

-- Scrollback
config.scrollback_lines = 10000

-- Shell
config.default_prog = { "/bin/zsh", "-l" }

-- Keys
config.keys = {
  -- Split panes
  { key = "|", mods = "CTRL|SHIFT", action = wezterm.action.SplitHorizontal { domain = "CurrentPaneDomain" } },
  { key = "-", mods = "CTRL|SHIFT", action = wezterm.action.SplitVertical   { domain = "CurrentPaneDomain" } },
  -- Navigate panes
  { key = "h", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Left")  },
  { key = "l", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Right") },
  { key = "k", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Up")    },
  { key = "j", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Down")  },
  -- New tab
  { key = "t", mods = "CTRL|SHIFT", action = wezterm.action.SpawnTab("CurrentPaneDomain") },
  -- Copy/paste
  { key = "c", mods = "CTRL|SHIFT", action = wezterm.action.CopyTo("Clipboard") },
  { key = "v", mods = "CTRL|SHIFT", action = wezterm.action.PasteFrom("Clipboard") },
}

return config
WEZCONF
echo "WezTerm installed with Catppuccin Mocha config"

# =============================================================================
echo "==> [7/8] Installing Discord..."
# =============================================================================
if [ "$ARCH" = "amd64" ]; then
    curl -Lo /tmp/discord.deb "https://discord.com/api/download?platform=linux&format=deb"
    sudo apt install -y /tmp/discord.deb
    rm /tmp/discord.deb
else
    echo "Discord: no official ARM64 build — install Vesktop instead:"
    echo "  https://github.com/Vencord/Vesktop/releases"
fi

# =============================================================================
echo "==> [8/8] Configuring shell..."
# =============================================================================

# tmux — TPM + config
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm 2>/dev/null || \
    git -C ~/.tmux/plugins/tpm pull

if [ ! -f ~/.tmux.conf ]; then
cat > ~/.tmux.conf << 'TMUXCONF'
# Prefix: Ctrl+a
unbind C-b
set -g prefix C-a
bind C-a send-prefix

set -g mouse on
set -g base-index 1
setw -g pane-base-index 1
set -g renumber-windows on
set -g history-limit 50000
set -sg escape-time 10
set -g default-terminal "tmux-256color"
set -ag terminal-overrides ",xterm-256color:RGB"

bind | split-window -h -c "#{pane_current_path}"
bind - split-window -v -c "#{pane_current_path}"
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R
bind r source-file ~/.tmux.conf \; display "Reloaded"

set -g @plugin 'tmux-plugins/tpm'
set -g @plugin 'tmux-plugins/tmux-sensible'
set -g @plugin 'catppuccin/tmux'
set -g @plugin 'tmux-plugins/tmux-yank'
set -g @catppuccin_flavour 'mocha'

run '~/.tmux/plugins/tpm/tpm'
TMUXCONF
fi
~/.tmux/plugins/tpm/bin/install_plugins

# zsh config
ZSHRC=~/.zshrc
add_line() { grep -qxF "$1" $ZSHRC 2>/dev/null || echo "$1" >> $ZSHRC; }

add_line 'eval "$(starship init zsh)"'
add_line 'eval "$(zoxide init zsh)"'
add_line 'alias ls="eza --icons"'
add_line 'alias ll="eza -la --icons --git"'
add_line 'alias lt="eza --tree --icons -L 2"'
add_line 'alias cat="bat"'
add_line 'alias lg="lazygit"'
add_line 'alias vim="nvim"'
add_line 'export PATH="$HOME/.local/bin:$PATH"'

# Set zsh as default shell
if [ "$SHELL" != "$(which zsh)" ]; then
    chsh -s $(which zsh)
    echo "Default shell changed to zsh — takes effect on next login"
fi

# =============================================================================
echo ""
echo "============================================================"
echo " Linux setup complete!"
echo "============================================================"
echo ""
echo " Next steps:"
echo "   1. Restart shell: exec zsh"
echo "   2. Open nvim to finish LazyVim install"
echo "   3. Start tmux: tmux new -s main"
echo "   4. Claude Code: claude"
echo "============================================================"
