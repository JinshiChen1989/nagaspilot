#!/usr/bin/env python3
"""
NPMapdBinaryManager - Enhanced binary management for pfeiferj/mapd integration
Based on proven patterns from existing NagasPilot MapdRunner and download systems
"""
import json
import requests
import os
import stat
import hashlib
import platform
import time
from pathlib import Path
from typing import Optional, Dict, Any
import subprocess
import signal

from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert

# Configuration
MAPD_BINARY_DIR = Path.home() / "nagaspilot" / "mapd" / "bin"
MAPD_VERSION_FILE = MAPD_BINARY_DIR / "version.json"
MAPD_BINARY_NAME = "mapd"

# GitHub release configuration for pfeiferj/mapd
GITHUB_API_BASE = "https://api.github.com/repos/pfeiferj/mapd"
GITHUB_RELEASES_URL = f"{GITHUB_API_BASE}/releases/latest"

# Platform mapping for binary downloads
PLATFORM_MAP = {
    "Linux": {
        "x86_64": "linux-amd64",
        "aarch64": "linux-arm64",
        "armv7l": "linux-armv7"
    },
    "Darwin": {
        "x86_64": "darwin-amd64",
        "arm64": "darwin-arm64"
    }
}


class NPMapdManager:
    """
    Enhanced binary manager for pfeiferj/mapd with auto-download and update capabilities.
    Extends proven MapdRunner patterns with GitHub integration and validation.
    """
    
    def __init__(self, params: Params):
        self.params = params
        self.proc: Optional[subprocess.Popen] = None
        self.last_path: Optional[str] = None
        self._ensure_directories()
    
    def _ensure_directories(self) -> None:
        """Create necessary directories for binary storage."""
        try:
            MAPD_BINARY_DIR.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
    
    def _get_platform_info(self) -> tuple[str, str]:
        """Get current platform and architecture for binary selection."""
        system = platform.system()
        machine = platform.machine()
        return system, machine
    
    def _get_binary_path(self) -> Path:
        """Get expected path for the mapd binary."""
        return MAPD_BINARY_DIR / MAPD_BINARY_NAME
    
    def _is_executable(self, path: Path) -> bool:
        """Check if file is executable using proven MapdRunner pattern."""
        try:
            st = path.stat()
            return stat.S_ISREG(st.st_mode) and os.access(path, os.X_OK)
        except Exception:
            return False
    
    def _calculate_file_hash(self, filepath: Path) -> str:
        """Calculate SHA256 hash of file for validation."""
        try:
            with open(filepath, 'rb') as f:
                return hashlib.sha256(f.read()).hexdigest()
        except Exception:
            return ""
    
    def _get_current_version_info(self) -> Dict[str, Any]:
        """Load current version info from local storage."""
        try:
            if MAPD_VERSION_FILE.exists():
                return json.loads(MAPD_VERSION_FILE.read_text())
        except Exception:
            pass
        return {}
    
    def _save_version_info(self, version_info: Dict[str, Any]) -> None:
        """Save version info to local storage."""
        try:
            MAPD_VERSION_FILE.write_text(json.dumps(version_info, indent=2))
        except Exception:
            pass
    
    def _get_latest_release_info(self) -> Optional[Dict[str, Any]]:
        """Fetch latest release info from GitHub API."""
        try:
            response = requests.get(GITHUB_RELEASES_URL, timeout=10)
            response.raise_for_status()
            return response.json()
        except Exception:
            return None
    
    def _find_asset_for_platform(self, assets: list) -> Optional[Dict[str, Any]]:
        """Find appropriate binary asset for current platform."""
        system, machine = self._get_platform_info()
        
        if system not in PLATFORM_MAP or machine not in PLATFORM_MAP[system]:
            return None
        
        platform_tag = PLATFORM_MAP[system][machine]
        
        for asset in assets:
            if platform_tag in asset.get("name", "").lower():
                return asset
        
        return None
    
    def _stream_download_binary(self, url: str, target_path: Path, expected_size: int = 0) -> bool:
        """
        Download binary with progress tracking using proven stream_download pattern.
        Adapted from existing np_mapd_manager.py.
        """
        try:
            mp = self._get_mem_params()
            with requests.get(url, stream=True, timeout=30) as r:
                r.raise_for_status()
                total = int(r.headers.get('Content-Length', expected_size))
                downloaded = 0
                
                # Initialize progress tracking
                mp.put("MapdBinaryDownloadProgress", json.dumps({
                    "total_files": 100, 
                    "downloaded_files": 0,
                    "status": "downloading"
                }))
                
                with open(target_path, 'wb') as f:
                    for chunk in r.iter_content(chunk_size=512 * 1024):
                        if not chunk:
                            continue
                        f.write(chunk)
                        downloaded += len(chunk)
                        
                        if total > 0:
                            pct = max(0, min(100, int(downloaded * 100 / total)))
                            mp.put("MapdBinaryDownloadProgress", json.dumps({
                                "total_files": 100, 
                                "downloaded_files": pct,
                                "status": "downloading"
                            }))
                
                # Set executable permissions
                target_path.chmod(0o755)
                
                # Complete progress tracking
                mp.put("MapdBinaryDownloadProgress", json.dumps({
                    "total_files": 100, 
                    "downloaded_files": 100,
                    "status": "completed"
                }))
                
                return True
                
        except Exception as e:
            try:
                mp = self._get_mem_params()
                mp.put("MapdBinaryDownloadProgress", json.dumps({
                    "total_files": 100, 
                    "downloaded_files": 0,
                    "status": f"error: {str(e)}"
                }))
            except Exception:
                pass
            return False
    
    def _get_mem_params(self) -> Params:
        """Get memory params using proven pattern from np_mapd_manager.py."""
        return Params("/dev/shm/params") if platform.system() != "Darwin" else Params()
    
    def check_for_updates(self) -> bool:
        """Check if binary update is available."""
        current_version = self._get_current_version_info()
        latest_release = self._get_latest_release_info()
        
        if not latest_release:
            return False
        
        current_tag = current_version.get("tag_name", "")
        latest_tag = latest_release.get("tag_name", "")
        
        return latest_tag != current_tag and latest_tag != ""
    
    def download_binary(self) -> bool:
        """Download and install the latest mapd binary."""
        release_info = self._get_latest_release_info()
        if not release_info:
            set_offroad_alert("Offroad_MapdBinaryUpdateFailed", True, 
                             "Failed to fetch latest mapd release info")
            return False
        
        asset = self._find_asset_for_platform(release_info.get("assets", []))
        if not asset:
            set_offroad_alert("Offroad_MapdBinaryUnsupported", True,
                             "No mapd binary available for your platform")
            return False
        
        download_url = asset.get("browser_download_url")
        if not download_url:
            return False
        
        # Download to temporary file first
        binary_path = self._get_binary_path()
        temp_path = binary_path.with_suffix(".tmp")
        
        try:
            if not self._stream_download_binary(download_url, temp_path, asset.get("size", 0)):
                return False
            
            # Validate downloaded binary
            if not self._is_executable(temp_path):
                temp_path.unlink(missing_ok=True)
                return False
            
            # Move to final location
            if binary_path.exists():
                binary_path.unlink()
            temp_path.rename(binary_path)
            
            # Save version info
            version_info = {
                "tag_name": release_info.get("tag_name", ""),
                "published_at": release_info.get("published_at", ""),
                "download_url": download_url,
                "file_hash": self._calculate_file_hash(binary_path),
                "installed_at": time.time()
            }
            self._save_version_info(version_info)
            
            # Update params to point to new binary
            self.params.put("NpMapdBinary", str(binary_path))
            
            # Clear any error alerts
            set_offroad_alert("Offroad_MapdBinaryUpdateFailed", False)
            set_offroad_alert("Offroad_MapdBinaryUnsupported", False)
            
            return True
            
        except Exception as e:
            # Cleanup on failure
            temp_path.unlink(missing_ok=True)
            set_offroad_alert("Offroad_MapdBinaryUpdateFailed", True,
                             f"Binary download failed: {str(e)}")
            return False
    
    def ensure_binary_available(self) -> bool:
        """Ensure mapd binary is available and up-to-date."""
        binary_path = self._get_binary_path()
        
        # Check if binary exists and is executable
        if not binary_path.exists() or not self._is_executable(binary_path):
            return self.download_binary()
        
        # Check if update is available (but don't force update)
        if self.check_for_updates():
            # Set parameter to indicate update available
            self.params.put_bool("MapdBinaryUpdateAvailable", True)
        else:
            self.params.put_bool("MapdBinaryUpdateAvailable", False)
        
        # Ensure params points to current binary
        current_path = self.params.get("NpMapdBinary", encoding='utf-8')
        if current_path != str(binary_path):
            self.params.put("NpMapdBinary", str(binary_path))
        
        return True
    
    def get_binary_info(self) -> Dict[str, Any]:
        """Get information about currently installed binary."""
        binary_path = self._get_binary_path()
        version_info = self._get_current_version_info()
        
        return {
            "path": str(binary_path),
            "exists": binary_path.exists(),
            "executable": self._is_executable(binary_path),
            "version_info": version_info,
            "update_available": self.check_for_updates()
        }
    
    def tick(self):
        """
        Main tick function for binary management.
        Extends MapdRunner.tick() with auto-download capability.
        """
        # Ensure binary is available
        if not self.ensure_binary_available():
            return
        
        # Use existing MapdRunner logic for process management
        want = self.params.get("NpMapdBinary", encoding='utf-8')
        
        # Restart if path changed
        if want != self.last_path:
            self.stop()
            self.last_path = want
        
        if want and Path(want).exists() and self._is_executable(Path(want)):
            if not self.proc or self.proc.poll() is not None:
                try:
                    # Run mapd binary with basic configuration
                    self.proc = subprocess.Popen(
                        [want], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL
                    )
                except Exception:
                    self.proc = None
        else:
            self.stop()
    
    def stop(self):
        """Stop running mapd process using proven pattern."""
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.send_signal(signal.SIGTERM)
                # Give it a moment to terminate gracefully
                time.sleep(0.1)
                if self.proc.poll() is None:
                    self.proc.kill()
            except Exception:
                pass
        self.proc = None


def main():
    """Main function for standalone testing."""
    params = Params()
    manager = NPMapdManager(params)
    
    print("NPMapdManager - Testing binary management")
    print(f"Binary info: {manager.get_binary_info()}")
    
    if manager.ensure_binary_available():
        print("✅ Binary is available and ready")
        manager.tick()  # Start the process
        time.sleep(2)   # Let it run briefly
        manager.stop()  # Clean shutdown
    else:
        print("❌ Failed to ensure binary availability")


if __name__ == "__main__":
    main()