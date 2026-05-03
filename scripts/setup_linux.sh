#!/bin/bash
# =============================================================================
# Linux Developer Setup Script (Ubuntu 22.04 / 24.04 / 26.04)
# Installs dev tools, apps, and configs.
# Usage: bash scripts/setup_linux.sh
# Does NOT abort on errors — collects all failures and reports at the end.
# =============================================================================

ARCH=$(dpkg --print-architecture)
UBUNTU_VERSION=$(lsb_release -rs)
FAILED_STEPS=()

echo "==> Ubuntu $UBUNTU_VERSION ($ARCH)"

# Run a named step; on failure, record it and continue.
run_step() {
    local name="$1"
    shift
    echo ""
    echo "==> [$name]"
    if "$@"; then
        echo "    OK: $name"
    else
        echo "    FAILED: $name (exit $?)"
        FAILED_STEPS+=("$name")
    fi
}

# =============================================================================
step_system_packages() {
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
    # fd is called fdfind on Ubuntu
    sudo ln -sf "$(which fdfind)" /usr/local/bin/fd 2>/dev/null || true
    # bat is called batcat on Ubuntu
    sudo ln -sf "$(which batcat)" /usr/local/bin/bat 2>/dev/null || true
}

step_nerd_font() {
    FONT_DIR=~/.local/share/fonts/JetBrainsMono
    mkdir -p "$FONT_DIR"
    NERD_VERSION=$(curl -sf https://api.github.com/repos/ryanoasis/nerd-fonts/releases/latest \
        | grep '"tag_name"' | cut -d'"' -f4)
    if [ -z "$NERD_VERSION" ]; then
        echo "Could not fetch nerd-fonts version"
        return 1
    fi
    curl -fLo /tmp/JetBrainsMono.zip \
        "https://github.com/ryanoasis/nerd-fonts/releases/download/${NERD_VERSION}/JetBrainsMono.zip"
    unzip -o /tmp/JetBrainsMono.zip "*.ttf" -d "$FONT_DIR"
    rm -f /tmp/JetBrainsMono.zip
    fc-cache -f
    echo "JetBrains Mono Nerd Font installed"
}

step_neovim() {
    case "$ARCH" in
        amd64)  NVIM_ARCH="x86_64" ;;
        arm64)  NVIM_ARCH="arm64" ;;
        *)      NVIM_ARCH="$ARCH" ;;
    esac
    NVIM_VERSION=$(curl -sf https://api.github.com/repos/neovim/neovim/releases/latest \
        | grep '"tag_name"' | cut -d'"' -f4)
    if [ -z "$NVIM_VERSION" ]; then
        echo "Could not fetch nvim version — falling back to apt"
        sudo apt install -y neovim
        return
    fi
    NVIM_TARBALL="nvim-linux-${NVIM_ARCH}.tar.gz"
    curl -fL --retry 3 -o "/tmp/${NVIM_TARBALL}" \
        "https://github.com/neovim/neovim/releases/download/${NVIM_VERSION}/${NVIM_TARBALL}"
    sudo rm -rf "/opt/nvim-linux-${NVIM_ARCH}"
    sudo tar -C /opt -xzf "/tmp/${NVIM_TARBALL}"
    sudo ln -sf "/opt/nvim-linux-${NVIM_ARCH}/bin/nvim" /usr/local/bin/nvim
    rm -f "/tmp/${NVIM_TARBALL}"
    echo "Neovim ${NVIM_VERSION} installed"
}

step_lazyvim() {
    [ -d ~/.config/nvim ] && mv ~/.config/nvim ~/.config/nvim.bak.$(date +%s)
    [ -d ~/.local/share/nvim ] && mv ~/.local/share/nvim ~/.local/share/nvim.bak.$(date +%s)
    git clone --depth 1 https://github.com/LazyVim/starter ~/.config/nvim
    rm -rf ~/.config/nvim/.git
    mkdir -p ~/.config/nvim/lua/plugins

    cat > ~/.config/nvim/lua/plugins/lazygit.lua << 'LUACONF'
return {
  {
    "folke/snacks.nvim",
    opts = {
      lazygit = { enabled = true },
    },
    keys = {
      { "<leader>gg", function() Snacks.lazygit() end,          desc = "Lazygit" },
      { "<leader>gG", function() Snacks.lazygit.log() end,      desc = "Lazygit log (cwd)" },
      { "<leader>gf", function() Snacks.lazygit.log_file() end, desc = "Lazygit current file" },
    },
  },
}
LUACONF

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
}

step_nodejs() {
    curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
    sudo apt install -y nodejs
    echo "Node $(node --version) installed"
}

step_claude_code() {
    sudo npm install -g @anthropic-ai/claude-code
    echo "Claude Code installed: $(claude --version 2>/dev/null || echo 'restart shell to verify')"
}

step_github_cli() {
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        | sudo tee /etc/apt/sources.list.d/github-cli.list
    sudo apt update && sudo apt install -y gh
    gh extension install dlvhdr/gh-dash
    echo "gh + gh-dash installed"
}

step_eza() {
    sudo mkdir -p /etc/apt/keyrings
    wget -qO- https://raw.githubusercontent.com/eza-community/eza/main/deb.asc \
        | sudo gpg --dearmor -o /etc/apt/keyrings/gierens.gpg
    echo "deb [signed-by=/etc/apt/keyrings/gierens.gpg] http://deb.gierens.de stable main" \
        | sudo tee /etc/apt/sources.list.d/gierens.list
    sudo chmod 644 /etc/apt/keyrings/gierens.gpg /etc/apt/sources.list.d/gierens.list
    sudo apt update && sudo apt install -y eza
}

step_zoxide() {
    curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh
}

step_starship() {
    curl -fsSL https://starship.rs/install.sh | sh -s -- --yes
}

step_lazygit() {
    LAZYGIT_VERSION=$(curl -sf https://api.github.com/repos/jesseduffield/lazygit/releases/latest \
        | grep '"tag_name"' | cut -d'"' -f4 | sed 's/v//')
    if [ -z "$LAZYGIT_VERSION" ]; then
        echo "Could not fetch lazygit version"
        return 1
    fi
    curl -fLo /tmp/lazygit.tar.gz \
        "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_${ARCH}.tar.gz"
    tar xf /tmp/lazygit.tar.gz -C /tmp lazygit
    sudo install /tmp/lazygit -D -t /usr/local/bin/
    rm -f /tmp/lazygit /tmp/lazygit.tar.gz
    echo "lazygit ${LAZYGIT_VERSION} installed"
}

step_wezterm() {
    curl -fsSL https://apt.fury.io/wez/gpg.key \
        | sudo gpg --yes --dearmor -o /usr/share/keyrings/wezterm-fury.gpg
    echo "deb [signed-by=/usr/share/keyrings/wezterm-fury.gpg] https://apt.fury.io/wez/ * *" \
        | sudo tee /etc/apt/sources.list.d/wezterm.list
    sudo apt update && sudo apt install -y wezterm

    mkdir -p ~/.config/wezterm
    cat > ~/.config/wezterm/wezterm.lua << 'WEZCONF'
local wezterm = require("wezterm")
local config = wezterm.config_builder()

config.font = wezterm.font("JetBrains Mono", { weight = "Regular" })
config.font_size = 13.0
config.color_scheme = "Catppuccin Mocha"
config.window_background_opacity = 0.95
config.window_padding = { left = 8, right = 8, top = 8, bottom = 8 }
config.enable_tab_bar = true
config.hide_tab_bar_if_only_one_tab = true
config.window_decorations = "TITLE | RESIZE"
config.scrollback_lines = 10000
config.default_prog = { "/bin/zsh", "-l" }

config.keys = {
  { key = "|", mods = "CTRL|SHIFT", action = wezterm.action.SplitHorizontal { domain = "CurrentPaneDomain" } },
  { key = "-", mods = "CTRL|SHIFT", action = wezterm.action.SplitVertical   { domain = "CurrentPaneDomain" } },
  { key = "h", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Left")  },
  { key = "l", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Right") },
  { key = "k", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Up")    },
  { key = "j", mods = "CTRL|SHIFT", action = wezterm.action.ActivatePaneDirection("Down")  },
  { key = "t", mods = "CTRL|SHIFT", action = wezterm.action.SpawnTab("CurrentPaneDomain") },
  { key = "c", mods = "CTRL|SHIFT", action = wezterm.action.CopyTo("Clipboard") },
  { key = "v", mods = "CTRL|SHIFT", action = wezterm.action.PasteFrom("Clipboard") },
}

return config
WEZCONF
    echo "WezTerm installed with Catppuccin Mocha config"
}

step_discord() {
    if [ "$ARCH" = "amd64" ]; then
        curl -fLo /tmp/discord.deb "https://discord.com/api/download?platform=linux&format=deb"
        sudo apt install -y /tmp/discord.deb
        rm -f /tmp/discord.deb
    else
        echo "No official ARM64 Discord — install Vesktop: https://github.com/Vencord/Vesktop/releases"
        return 1
    fi
}

step_tmux_config() {
    git clone --depth 1 https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm 2>/dev/null || \
        git -C ~/.tmux/plugins/tpm pull

    if [ ! -f ~/.tmux.conf ]; then
        cat > ~/.tmux.conf << 'TMUXCONF'
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
}

step_shell_config() {
    ZSHRC=~/.zshrc
    add_line() { grep -qxF "$1" "$ZSHRC" 2>/dev/null || echo "$1" >> "$ZSHRC"; }

    add_line 'export PATH="$HOME/.local/bin:$PATH"'
    add_line 'eval "$(starship init zsh)"'
    add_line 'eval "$(zoxide init zsh)"'
    add_line 'alias ls="eza --icons"'
    add_line 'alias ll="eza -la --icons --git"'
    add_line 'alias lt="eza --tree --icons -L 2"'
    add_line 'alias cat="bat"'
    add_line 'alias lg="lazygit"'
    add_line 'alias vim="nvim"'

    if [ "$SHELL" != "$(which zsh)" ]; then
        chsh -s "$(which zsh)"
        echo "Default shell changed to zsh — takes effect on next login"
    fi
}

# =============================================================================
# Run all steps
# =============================================================================
run_step "System packages"        step_system_packages
run_step "JetBrains Mono font"    step_nerd_font
run_step "Neovim (latest)"        step_neovim
run_step "LazyVim"                step_lazyvim
run_step "Node.js"                step_nodejs
run_step "Claude Code"            step_claude_code
run_step "GitHub CLI + gh-dash"   step_github_cli
run_step "eza"                    step_eza
run_step "zoxide"                 step_zoxide
run_step "starship"               step_starship
run_step "lazygit"                step_lazygit
run_step "WezTerm"                step_wezterm
run_step "Discord"                step_discord
run_step "tmux + TPM"             step_tmux_config
run_step "Shell config (zsh)"     step_shell_config

# =============================================================================
echo ""
echo "============================================================"
if [ ${#FAILED_STEPS[@]} -eq 0 ]; then
    echo " Linux setup complete — all steps passed!"
else
    echo " Linux setup complete with ${#FAILED_STEPS[@]} failure(s):"
    for s in "${FAILED_STEPS[@]}"; do
        echo "   ✗  $s"
    done
fi
echo "============================================================"
echo ""
echo " Next steps:"
echo "   1. Restart shell: exec zsh"
echo "   2. Open nvim to finish LazyVim install"
echo "   3. Start tmux: tmux new -s main"
echo "   4. Claude Code: claude"
echo "============================================================"
