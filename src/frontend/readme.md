# Robot Link Forge - Frontend

This directory contains the frontend application for the Robot Link Forge, built with React, TypeScript, Vite, and Three.js.

## Development Setup

### 1. Prerequisites: Node.js

It is highly recommended to use [Node Version Manager (nvm)](https://github.com/nvm-sh/nvm) to manage your Node.js versions. This project requires **Node.js v18 or newer**.

**Install nvm:**
```bash
# Using curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash

# Or using wget
wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
```
After installation, close and reopen your terminal.

**Troubleshooting: `nvm: command not found`**

If you open a new terminal after installation and see `nvm: command not found`, it means the nvm script was not correctly added to your shell's startup file. You can fix this by manually adding the following lines to your shell configuration file (e.g., `~/.bashrc`, `~/.zshrc`):

```bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
```

Then, reload your shell configuration by running `source ~/.bashrc` (or your respective file, like `source ~/.zshrc`) or by opening a new terminal.

**Install and Use Node.js:**
```bash
# Install the latest Long-Term Support (LTS) version of Node.js
nvm install --lts

# Set the LTS version as the one to use
nvm use --lts
```

To verify your Node.js version, run:
```bash
node -v
# Should output v18.x.x or higher
```

### 2. Install Dependencies

Navigate to this directory and install the required npm packages.

```bash
cd src/frontend
npm install
```

### 3. Run the Development Server

Once the dependencies are installed, you can start the Vite development server.

```bash
npm run dev
```

The application will be available at `http://localhost:5173`.
