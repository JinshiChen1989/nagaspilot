#!/usr/bin/env python3
"""
NagasPilot Log Mapper - Map NagasPilot and OpenPilot logs by timestamp
Shows the relationship between swaglog and np_* log files for analysis

Usage:
    python3 np_log_mapper.py --time "2025-01-24 10:30:00" --duration 5m
    python3 np_log_mapper.py --list-files
    python3 np_log_mapper.py --correlation-analysis
"""

import os
import glob
import argparse
from datetime import datetime, timedelta
import re

LOG_DIR = "/data/log"
MEDIA_DIR = "/data/media/0/realdata"

class NpLogMapper:
    def __init__(self):
        self.log_dir = LOG_DIR
        self.media_dir = MEDIA_DIR
        
    def list_all_logs(self):
        """List all log files with their creation times"""
        print(f"\n=== Log Files in {self.log_dir} ===")
        
        # OpenPilot swaglog files
        swaglog_pattern = os.path.join(self.log_dir, "swaglog.*")
        swaglog_files = sorted(glob.glob(swaglog_pattern))
        
        # NagasPilot module files
        np_pattern = os.path.join(self.log_dir, "np_*.*")
        np_files = sorted(glob.glob(np_pattern))
        
        print(f"\nOpenPilot swaglog files: {len(swaglog_files)}")
        for log_file in swaglog_files[-5:]:  # Show last 5
            try:
                mtime = os.path.getmtime(log_file)
                timestamp = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
                size = os.path.getsize(log_file)
                print(f"  {os.path.basename(log_file)} - {timestamp} ({size:,} bytes)")
            except:
                print(f"  {os.path.basename(log_file)} - (error reading)")
        
        print(f"\nNagasPilot module files: {len(np_files)}")
        modules = {}
        for log_file in np_files:
            module = os.path.basename(log_file).split('.')[0]  # np_lca, np_hod, etc.
            if module not in modules:
                modules[module] = []
            modules[module].append(log_file)
        
        for module, files in modules.items():
            print(f"  {module}: {len(files)} files")
            if files:
                latest = max(files, key=os.path.getmtime)
                try:
                    mtime = os.path.getmtime(latest)
                    timestamp = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
                    size = os.path.getsize(latest)
                    print(f"    Latest: {os.path.basename(latest)} - {timestamp} ({size:,} bytes)")
                except:
                    print(f"    Latest: {os.path.basename(latest)} - (error reading)")
    
    def find_logs_by_time(self, target_time, duration_minutes=5):
        """Find all log files (swaglog and np_*) active during a time period"""
        start_time = target_time - timedelta(minutes=duration_minutes//2)
        end_time = target_time + timedelta(minutes=duration_minutes//2)
        
        print(f"\n=== Logs Active Between {start_time} and {end_time} ===")
        
        # Find swaglog files in time range
        swaglog_pattern = os.path.join(self.log_dir, "swaglog.*")
        swaglog_files = glob.glob(swaglog_pattern)
        
        active_swaglogs = []
        for log_file in swaglog_files:
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(log_file))
                if start_time <= mtime <= end_time:
                    active_swaglogs.append((log_file, mtime))
            except:
                continue
        
        # Find np_* files in time range
        np_pattern = os.path.join(self.log_dir, "np_*.*")
        np_files = glob.glob(np_pattern)
        
        active_np_logs = {}
        for log_file in np_files:
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(log_file))
                if start_time <= mtime <= end_time:
                    module = os.path.basename(log_file).split('.')[0]
                    if module not in active_np_logs:
                        active_np_logs[module] = []
                    active_np_logs[module].append((log_file, mtime))
            except:
                continue
        
        # Display results
        print(f"\nOpenPilot swaglog files: {len(active_swaglogs)}")
        for log_file, mtime in sorted(active_swaglogs, key=lambda x: x[1]):
            print(f"  {os.path.basename(log_file)} - {mtime.strftime('%Y-%m-%d %H:%M:%S')}")
        
        print(f"\nNagasPilot module files:")
        for module in sorted(active_np_logs.keys()):
            files = active_np_logs[module]
            print(f"  {module}: {len(files)} files")
            for log_file, mtime in sorted(files, key=lambda x: x[1]):
                print(f"    {os.path.basename(log_file)} - {mtime.strftime('%Y-%m-%d %H:%M:%S')}")
        
        return active_swaglogs, active_np_logs
    
    def correlation_analysis(self):
        """Analyze correlation between swaglog and np_* file creation"""
        print(f"\n=== Correlation Analysis ===")
        
        # Get all files with timestamps
        swaglog_pattern = os.path.join(self.log_dir, "swaglog.*")
        swaglog_files = glob.glob(swaglog_pattern)
        
        np_pattern = os.path.join(self.log_dir, "np_*.*")
        np_files = glob.glob(np_pattern)
        
        # Create timeline
        timeline = []
        
        for log_file in swaglog_files:
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(log_file))
                timeline.append((mtime, 'swaglog', os.path.basename(log_file)))
            except:
                continue
        
        for log_file in np_files:
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(log_file))
                module = os.path.basename(log_file).split('.')[0]
                timeline.append((mtime, module, os.path.basename(log_file)))
            except:
                continue
        
        # Sort by time and show recent activity
        timeline.sort()
        
        print(f"\nRecent log file activity (last 20 files):")
        for mtime, log_type, filename in timeline[-20:]:
            print(f"  {mtime.strftime('%Y-%m-%d %H:%M:%S')} - {log_type:12} - {filename}")
        
        # Analyze rotation patterns
        print(f"\nRotation Pattern Analysis:")
        print(f"  Total swaglog files: {len([x for x in timeline if x[1] == 'swaglog'])}")
        
        modules = set([x[1] for x in timeline if x[1].startswith('np_')])
        for module in sorted(modules):
            count = len([x for x in timeline if x[1] == module])
            print(f"  Total {module} files: {count}")
    
    def find_camera_files(self, target_time, duration_minutes=60):
        """Find camera/video files corresponding to log time period"""
        start_time = target_time - timedelta(minutes=duration_minutes//2)
        end_time = target_time + timedelta(minutes=duration_minutes//2)
        
        print(f"\n=== Camera/Video Files ({start_time} to {end_time}) ===")
        
        if not os.path.exists(self.media_dir):
            print(f"Camera directory not found: {self.media_dir}")
            return
        
        # Look for video files in media directory
        video_extensions = ['*.mp4', '*.h264', '*.hevc']
        camera_files = []
        
        for ext in video_extensions:
            pattern = os.path.join(self.media_dir, f"**/{ext}")
            files = glob.glob(pattern, recursive=True)
            camera_files.extend(files)
        
        # Filter by time range
        matching_videos = []
        for video_file in camera_files:
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(video_file))
                if start_time <= mtime <= end_time:
                    matching_videos.append((video_file, mtime))
            except:
                continue
        
        if matching_videos:
            print(f"Found {len(matching_videos)} video files:")
            for video_file, mtime in sorted(matching_videos, key=lambda x: x[1]):
                size_mb = os.path.getsize(video_file) / (1024*1024)
                print(f"  {os.path.basename(video_file)} - {mtime.strftime('%Y-%m-%d %H:%M:%S')} ({size_mb:.1f}MB)")
            
            print(f"\n=== Camera Analysis Commands ===")
            print(f"# Extract frames from video at specific time:")
            latest_video = sorted(matching_videos, key=lambda x: x[1])[-1][0]
            print(f"  ffmpeg -i {latest_video} -ss 00:01:00 -t 00:00:10 -vf fps=1 frame_%03d.png")
            print(f"# Play video with timestamp overlay:")
            print(f"  ffplay {latest_video}")
            print(f"# Get video info:")
            print(f"  ffprobe -v quiet -show_entries format=duration -of csv=p=0 {latest_video}")
        else:
            print("No video files found in time range")
            
        # Also check for cereal logs that contain camera frame metadata
        self.find_camera_metadata(start_time, end_time)
    
    def find_camera_metadata(self, start_time, end_time):
        """Find camera frame metadata in cereal logs"""
        print(f"\n=== Camera Metadata (cereal logs) ===")
        
        # Look for rlog files (cereal logs)
        rlog_pattern = os.path.join(self.log_dir, "**/*.rlog")
        rlog_files = glob.glob(rlog_pattern, recursive=True)
        
        if rlog_files:
            print(f"Found {len(rlog_files)} cereal log files")
            print(f"# Extract camera frames from cereal logs:")
            for rlog_file in rlog_files[:3]:  # Show first 3
                print(f"  # Camera frame data in: {os.path.basename(rlog_file)}")
            print(f"# Use tools/replay/route.py or cabana to analyze camera data")
        else:
            print("No cereal log files found")
            print("Note: Camera frames are typically in /data/media/0/realdata/ as video files")
    
    def generate_mapping_report(self, target_time, duration_minutes=60):
        """Generate a detailed mapping report for analysis"""
        print(f"\n=== Mapping Report for {target_time} (±{duration_minutes//2}min) ===")
        
        active_swaglogs, active_np_logs = self.find_logs_by_time(target_time, duration_minutes)
        
        if not active_swaglogs and not active_np_logs:
            print("No log files found in the specified time range.")
            return
        
        print(f"\n=== Analysis Commands ===")
        
        # Commands to analyze swaglog files
        if active_swaglogs:
            print(f"\nAnalyze OpenPilot system logs:")
            for log_file, mtime in active_swaglogs[:3]:  # Show first 3 commands
                print(f"  grep 'LCA\\|HOD\\|SSD\\|MTSC\\|VTSC\\|VCSC\\|PDA\\|DCP\\|DLP' {log_file} | head -20")
            
        # Commands to analyze NagasPilot files
        if active_np_logs:
            print(f"\nAnalyze NagasPilot module logs:")
            for module in sorted(list(active_np_logs.keys())[:3]):  # Show first 3 modules
                files = active_np_logs[module]
                if files:
                    latest_file = max(files, key=lambda x: x[1])[0]
                    print(f"  tail -50 {latest_file}")
        
        print(f"\nCombined analysis:")
        print(f"  # Find all NP logs in swaglog files during this time")
        if active_swaglogs:
            swaglog_list = ' '.join([f[0] for f in active_swaglogs[:5]])
            print(f"  grep 'LCA\\|HOD\\|SSD\\|MTSC\\|VTSC\\|VCSC\\|PDA\\|DCP\\|DLP' {swaglog_list} | sort")
        
        # Find corresponding camera/media files
        self.find_camera_files(target_time, duration_minutes)

def main():
    parser = argparse.ArgumentParser(description='Map NagasPilot and OpenPilot logs by timestamp')
    parser.add_argument('--time', help='Target time (YYYY-MM-DD HH:MM:SS)')
    parser.add_argument('--duration', help='Duration around target time (e.g., 5m, 1h)', default='30m')
    parser.add_argument('--list-files', action='store_true', help='List all log files')
    parser.add_argument('--correlation-analysis', action='store_true', help='Analyze log correlation')
    
    args = parser.parse_args()
    
    mapper = NpLogMapper()
    
    if args.list_files:
        mapper.list_all_logs()
    elif args.correlation_analysis:
        mapper.correlation_analysis()
    elif args.time:
        try:
            target_time = datetime.strptime(args.time, '%Y-%m-%d %H:%M:%S')
            
            # Parse duration
            duration_minutes = 30  # default
            if args.duration.endswith('m'):
                duration_minutes = int(args.duration[:-1])
            elif args.duration.endswith('h'):
                duration_minutes = int(args.duration[:-1]) * 60
            
            mapper.generate_mapping_report(target_time, duration_minutes)
        except ValueError:
            print("Invalid time format. Use: YYYY-MM-DD HH:MM:SS")
    else:
        parser.print_help()

if __name__ == '__main__':
    main()