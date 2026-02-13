#!/usr/bin/env python3
"""
Simulate command execution times based on historical data from command_log.db
"""

import sqlite3
import os

DEFAULT_DURATION = 4.0  # Default duration if no history exists
INITIALIZATION_OVERHEAD = 2.0  # Time for EV initialization

def get_average_duration(cursor, command, parameter):
    """
    Get the average duration for a command with specific parameters from the database.
    Returns (average_duration, count) or (None, 0) if no history exists.
    """
    cursor.execute(
        'SELECT AVG(duration), COUNT(*) FROM command_log WHERE command = ? AND parameter = ?',
        (command, parameter)
    )
    result = cursor.fetchone()
    if result and result[1] > 0:
        return result[0], result[1]
    return None, 0


def simulate_time(filename="commands.txt"):
    """
    Reads commands from a text file and estimates execution time based on historical data.
    """
    db_path = "command_log.db"
    
    # Check if database exists
    if not os.path.exists(db_path):
        print(f"Warning: Database '{db_path}' not found. Using default durations.")
        conn = None
        cursor = None
    else:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
    
    total_duration = 0.0
    command_count = 0
    warnings = []
    
    print("=" * 60)
    print("COMMAND EXECUTION TIME SIMULATION")
    print("=" * 60)
    print(f"{'Initialization overhead':<40} {INITIALIZATION_OVERHEAD:>12.2f}")
    print("-" * 60)
    print(f"{'Command':<40} {'Duration (s)':>12}")
    print("-" * 60)
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                    
                parts = line.lower().split()
                if len(parts) == 0:
                    continue
                    
                cmd = parts[0]
                param = ' '.join(parts[1:]) if len(parts) > 1 else ''
                
                # Parse command to get display string (similar to rt.py)
                display_cmd = None
                
                if cmd == 'move':
                    if len(parts) == 1:
                        display_cmd = f"MOVE 50cm in 3s"
                    elif len(parts) == 2:
                        display_cmd = f"MOVE {parts[1]}cm in 50s"
                    elif len(parts) >= 3:
                        display_cmd = f"MOVE {parts[1]}cm in {parts[2]}s"
                        
                elif cmd == 'forward':
                    if len(parts) == 1:
                        block = 1
                    else:
                        try:
                            block = float(parts[1])
                        except ValueError:
                            block = 1
                    dist = block * 50  # DEFAULT_MOVE_BLOCK = 50
                    display_cmd = f"FORWARD {block} blocks ({dist}cm)"
                    
                elif cmd == 'back':
                    if len(parts) == 1:
                        block = 1
                    else:
                        try:
                            block = float(parts[1])
                        except ValueError:
                            block = 1
                    dist = block * 50
                    display_cmd = f"BACK {block} blocks ({dist}cm)"
                    
                elif cmd == 'left':
                    display_cmd = "LEFT (CCW 90°)"
                    
                elif cmd == 'right':
                    display_cmd = "RIGHT (CW 90°)"
                    
                else:
                    # Unrecognized command - skip
                    continue
                
                # Get average duration from database
                if cursor:
                    avg_duration, count = get_average_duration(cursor, cmd, param)
                else:
                    avg_duration, count = None, 0
                
                if avg_duration is not None:
                    duration = avg_duration
                    duration_str = f"{duration:.2f}"
                    history_info = f"(avg of {count})"
                else:
                    duration = DEFAULT_DURATION
                    duration_str = f"{duration:.2f}*"
                    history_info = "(default)"
                    warnings.append(f"  - {cmd} '{param}': No history found, using default {DEFAULT_DURATION}s")
                
                total_duration += duration
                command_count += 1
                
                # Print command with duration
                cmd_display = f"{display_cmd} {history_info}"
                print(f"{cmd_display:<40} {duration_str:>12}")
                
    except FileNotFoundError:
        print(f"Error: Command file '{filename}' not found.")
        if conn:
            conn.close()
        return
    
    total_duration += INITIALIZATION_OVERHEAD
    
    print("-" * 60)
    print(f"{'TOTAL ESTIMATED TIME':<40} {total_duration:>12.2f}")
    print(f"{'Commands to execute:':<40} {command_count:>12}")
    print("=" * 60)
    
    # Print warnings if any
    if warnings:
        print("\nWARNINGS (commands without history, using default duration):")
        for warning in warnings:
            print(warning)
        print(f"\n* Commands marked with * use the default duration of {DEFAULT_DURATION}s")
    
    if conn:
        conn.close()
    
    return total_duration


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        simulate_time(sys.argv[1])
    else:
        simulate_time()
