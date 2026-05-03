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
