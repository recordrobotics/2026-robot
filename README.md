<h1 align="center">
  2025 Control

  <p align="center">
      <a href="https://github.com/recordrobotics/2025_Control/actions/workflows/ci.yml"><img alt="CI" src="https://github.com/recordrobotics/2025_Control/actions/workflows/ci.yml/badge.svg?branch=main"></a>
      <a href="https://docs.recordrobotics.org/"><img alt="Read the Docs" src="https://img.shields.io/readthedocs/2024-control?logo=readthedocs&labelColor=%23556bc2"></a>
      <a href="https://github.com/recordrobotics/2025_Control/actions/workflows/ci.yml"><img alt="Unit Tests Status" src="https://img.shields.io/github/check-runs/recordrobotics/2025_Control/main?nameFilter=JUnit%20Test%20Report&logo=gradle&label=tests&labelColor=purple"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics_2025_Control"><img alt="Quality Gate Status" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics_2025_Control&metric=alert_status"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics_2025_Control"><img alt="Coverage" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics_2025_Control&metric=coverage"></a>
  </p>
</h1>

Here are our [goals](https://recordrobotics.notion.site/1714851f43d58095ac37c44f40ad3b70?v=1714851f43d5800d9462000c01589958) for the season.

Docs at [docs.recordrobotics.org](https://docs.recordrobotics.org/)

---

## Extensions

This repository recommends the following VS Code extensions for the best development experience:

<table>
  <tr>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker">
        <img src="https://streetsidesoftware.gallerycdn.vsassets.io/extensions/streetsidesoftware/code-spell-checker/4.2.3/1753028947698/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Code Spell Checker"/><br/>
        <b>Code Spell Checker</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=SonarSource.sonarlint-vscode">
        <img src="https://sonarsource.gallerycdn.vsassets.io/extensions/sonarsource/sonarlint-vscode/4.29.0/1755515927519/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="SonarLint"/><br/>
        <b>SonarLint</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=shengchen.vscode-checkstyle">
        <img src="https://shengchen.gallerycdn.vsassets.io/extensions/shengchen/vscode-checkstyle/1.4.2/1680054146028/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Checkstyle for Java"/><br/>
        <b>Checkstyle for Java</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=edonet.vscode-command-runner">
        <img src="https://edonet.gallerycdn.vsassets.io/extensions/edonet/vscode-command-runner/0.0.124/1680934084102/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Command Runner"/><br/>
        <b>Command Runner</b>
      </a>
    </td>
  </tr>
</table>

### SonarLint Setup

To use the custom SonarLint rules configured for this repository, you need to copy the SonarLint rules from [`.vscode/settings.json`](.vscode/settings.json) into your **user settings**:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open User Settings (JSON)**.
3. Copy the `"sonarlint.rules"` section from [`.vscode/settings.json`](.vscode/settings.json) into your user settings file.

This ensures SonarLint uses the same code quality rules as the repository.

### How to use Command Runner

This repository is set up to show useful commands in a menu when pressing `Ctrl+Shift+R`

<img width="1244" height="160" alt="image" src="https://github.com/user-attachments/assets/bbc890e2-5ade-4d3b-a0a5-5e3c6b319964" />

#### Available commands

- **tuning swerve encoders** — when selecting this command an additional window prompting for the type of module appears. The options are `kraken|falcon` with default `kraken`

  <img width="996" height="141" alt="image" src="https://github.com/user-attachments/assets/5431c4f5-0d50-4350-90ad-8c85f21207a4" />

#### Keybindings

An example keybinding is provided in [`.vscode/keybindings.json`](.vscode/keybindings.json), allowing you to set custom keybinds to certain commands.

**IMPORTANT:** to add keybinds:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open Keyboard Shortcuts (JSON)**.
3. This is where you can define the custom keybinds (**this applies for your whole vscode user not just this repo**)

-------------------------------------------------
[Record Robotics](https://www.recordrobotics.org/)
