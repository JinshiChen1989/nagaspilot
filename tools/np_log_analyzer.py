#!/usr/bin/env python3
"""
NagasPilot Log Analyzer
Analyze and visualize NagasPilot module logs

Usage:
    python3 np_log_analyzer.py --module lca --duration 30m
    python3 np_log_analyzer.py --all --export csv
    python3 np_log_analyzer.py --plot --module mtsc
"""

import os
import glob
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import re

LOG_DIR = "/data/log"  # Same directory as OpenPilot swaglog

class NpLogAnalyzer:
    def __init__(self):
        self.modules = ['lca', 'hod', 'ssd', 'mtsc', 'vtsc', 'vcsc', 'dcp', 'pda', 'soc', 'vrc', 'dlp']
        
    def parse_log_line(self, line):
        """Parse a log line into structured data"""
        # Format: 2025-01-24 10:30:15 [DEBUG] [NP_LCA] message
        pattern = r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \[(\w+)\] \[NP_(\w+)\] (.+)'
        match = re.match(pattern, line.strip())
        
        if match:
            timestamp_str, level, module, message = match.groups()
            timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
            return {
                'timestamp': timestamp,
                'level': level,
                'module': module.lower(),
                'message': message
            }
        return None
    
    def read_module_logs(self, module, duration_minutes=None):
        """Read logs for a specific module"""
        pattern = os.path.join(LOG_DIR, f"np_{module}.*")
        log_files = sorted(glob.glob(pattern))
        
        if not log_files:
            print(f"No log files found for module: {module}")
            return pd.DataFrame()
        
        all_logs = []
        cutoff_time = None
        if duration_minutes:
            cutoff_time = datetime.now() - timedelta(minutes=duration_minutes)
        
        for log_file in log_files[-100:]:  # Last 100 files (matching OpenPilot retention)
            try:
                with open(log_file, 'r') as f:
                    for line in f:
                        parsed = self.parse_log_line(line)
                        if parsed:
                            if cutoff_time and parsed['timestamp'] < cutoff_time:
                                continue
                            all_logs.append(parsed)
            except Exception as e:
                print(f"Error reading {log_file}: {e}")
        
        return pd.DataFrame(all_logs)
    
    def analyze_module(self, module, duration_minutes=None):
        """Analyze logs for a specific module"""
        df = self.read_module_logs(module, duration_minutes)
        
        if df.empty:
            print(f"No data found for module: {module}")
            return
        
        print(f"\n=== {module.upper()} Log Analysis ===")
        print(f"Time range: {df['timestamp'].min()} to {df['timestamp'].max()}")
        print(f"Total log entries: {len(df)}")
        
        # Log level breakdown
        level_counts = df['level'].value_counts()
        print(f"\nLog levels:")
        for level, count in level_counts.items():
            print(f"  {level}: {count}")
        
        # Recent critical messages
        critical = df[df['level'].isin(['WARNING', 'ERROR'])].tail(10)
        if not critical.empty:
            print(f"\nRecent warnings/errors:")
            for _, row in critical.iterrows():
                print(f"  {row['timestamp']} [{row['level']}] {row['message'][:80]}...")
        
        return df
    
    def plot_module_activity(self, module, duration_minutes=60):
        """Plot module log activity over time"""
        df = self.read_module_logs(module, duration_minutes)
        
        if df.empty:
            print(f"No data to plot for module: {module}")
            return
        
        # Create time-based bins (5-minute intervals)
        df['time_bin'] = df['timestamp'].dt.floor('5T')
        activity = df.groupby(['time_bin', 'level']).size().unstack(fill_value=0)
        
        plt.figure(figsize=(12, 6))
        activity.plot(kind='bar', stacked=True, ax=plt.gca())
        plt.title(f'{module.upper()} Log Activity (Last {duration_minutes} minutes)')
        plt.xlabel('Time (5-minute bins)')
        plt.ylabel('Log Count')
        plt.xticks(rotation=45)
        plt.legend(title='Log Level')
        plt.tight_layout()
        plt.show()
    
    def export_csv(self, module, filename=None):
        """Export module logs to CSV"""
        df = self.read_module_logs(module)
        
        if df.empty:
            print(f"No data to export for module: {module}")
            return
        
        if not filename:
            filename = f"{module}_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        df.to_csv(filename, index=False)
        print(f"Exported {len(df)} log entries to {filename}")
    
    def live_tail(self, module):
        """Live tail of module logs (like tail -f)"""
        pattern = os.path.join(LOG_DIR, f"np_{module}.*")
        
        print(f"Live tailing {module.upper()} logs (Ctrl+C to stop)...")
        
        try:
            import time
            last_files = set()
            
            while True:
                current_files = set(glob.glob(pattern))
                new_files = current_files - last_files
                
                # Check all current files for new content
                for log_file in sorted(current_files)[-2:]:  # Last 2 files
                    try:
                        with open(log_file, 'r') as f:
                            lines = f.readlines()
                            for line in lines[-5:]:  # Show last 5 lines
                                parsed = self.parse_log_line(line)
                                if parsed:
                                    print(f"{parsed['timestamp']} [{parsed['level']}] {parsed['message']}")
                    except Exception:
                        pass
                
                last_files = current_files
                time.sleep(2)  # Check every 2 seconds
                
        except KeyboardInterrupt:
            print("\nStopped live tail")

def main():
    parser = argparse.ArgumentParser(description='NagasPilot Log Analyzer')
    parser.add_argument('--module', choices=['lca', 'hod', 'ssd', 'mtsc', 'vtsc', 'vcsc', 'dcp', 'pda', 'soc', 'vrc', 'dlp', 'all'])
    parser.add_argument('--duration', help='Duration to analyze (e.g., 30m, 2h)', default='60m')
    parser.add_argument('--plot', action='store_true', help='Generate activity plot')
    parser.add_argument('--export', choices=['csv'], help='Export format')
    parser.add_argument('--tail', action='store_true', help='Live tail logs')
    
    args = parser.parse_args()
    
    analyzer = NpLogAnalyzer()
    
    # Parse duration
    duration_minutes = 60  # default
    if args.duration.endswith('m'):
        duration_minutes = int(args.duration[:-1])
    elif args.duration.endswith('h'):
        duration_minutes = int(args.duration[:-1]) * 60
    
    if args.module == 'all':
        for module in analyzer.modules:
            analyzer.analyze_module(module, duration_minutes)
    elif args.module:
        if args.tail:
            analyzer.live_tail(args.module)
        elif args.plot:
            analyzer.plot_module_activity(args.module, duration_minutes)
        elif args.export:
            analyzer.export_csv(args.module)
        else:
            analyzer.analyze_module(args.module, duration_minutes)
    else:
        parser.print_help()

if __name__ == '__main__':
    main()