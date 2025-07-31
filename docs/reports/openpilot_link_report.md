# Remote Connections Report

This report details remote connections found in the codebase related to "openpilot" or "dragonpilot" URLs and GitHub.

## Found Connections:

- **File:** [`system/ui/setup.py:28`](file:///home/vcar/Winsurf/openpilot/system/ui/setup.py#L28)
  - **URL:** `https://openpilot.comma.ai`

- **File:** [`selfdrive/ui/qt/setup/setup.cc:23`](file:///home/vcar/Winsurf/openpilot/selfdrive/ui/qt/setup/setup.cc#L23)
  - **URL:** `https://openpilot.comma.ai`

These URLs point to the official openpilot website. While no direct GitHub links or "dragonpilot" specific URLs were found in the initial search for remote connections, further investigation revealed that "dragonpilot" is extensively integrated throughout the codebase. It appears in:

- **License and Changelogs:** Indicating its historical and licensing significance.
- **Code and Configuration:** Found in Python and C++ source files, often in imports, file paths, and configuration settings.
- **UI Assets:** References to "dragonpilot" assets (e.g., `dragonpilot.png`, `QR.png`, icons) are present in UI-related files.
- **Debug/Informational Messages:** Some `print` statements also contain "dragonpilot".
- **Dedicated Directory:** A `dragonpilot/` directory exists, containing its own source code and assets.

This indicates that "dragonpilot" is a significant and integrated part of this openpilot fork.