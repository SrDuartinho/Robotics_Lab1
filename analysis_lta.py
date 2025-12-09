"""
LTA Controller Effectiveness Analysis

Computes lane-keeping metrics from CSV logs:
- % time in lane (lane-keeping success rate)
- Number and duration of lane departures
- Average lateral error during departures
- Speed and steering statistics per condition
"""
from constants import * 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def load_log(log_path):
    """Load CSV log and return DataFrame."""
    try:
        df = pd.read_csv(log_path)
        print(f"Loaded {len(df)} frames from {log_path}")
        return df
    except FileNotFoundError:
        print(f"Error: Log file not found: {log_path}")
        return None


def compute_lane_keeping_metrics(df, lane_threshold_px=40.0):
    """
    Compute lane-keeping effectiveness metrics.
    
    lane_threshold_px: distance in pixels beyond which car is "out of lane"
    (typically LTA_THRESHOLD, which is 40 px in your constants)
    """
    # Remove rows with NaN lateral distances
    valid = df[df['lateral_dist_px'].notna()].copy()
    
    if len(valid) == 0:
        print("No valid lateral distance data.")
        return {}
    
    # Evaluate effectiveness only when LTA is active
    active = valid[valid['lta_active'].astype(bool)]
    if len(active) == 0:
        print("No frames with LTA active; skipping effectiveness calculation.")
        return {
            'total_frames': len(valid),
            'duration_s': 0.0,
            'pct_in_lane': np.nan,
            'num_active_segments': 0,
            'successful_segments': 0,
            'num_departures': 0,
            'avg_lateral_error_px': np.nan,
            'max_lateral_error_px': np.nan,
            'rms_lateral_error_px': np.nan,
            'avg_lateral_error_m': np.nan,
            'max_lateral_error_m': np.nan,
            'rms_lateral_error_m': np.nan,
            'avg_steering_rad': np.nan,
            'max_steering_rad': np.nan,
            'avg_speed_pps': np.nan,
            'avg_speed_mps': np.nan,
            'lta_active_pct': 0.0,
        }
    
    # Segment-level success: for each contiguous LTA-active segment, effectiveness is 100% if
    # the car never touches the line (lateral_dist_px stays > CAR_WIDTH_PX/2); otherwise 0%.
    active_mask = valid['lta_active'].astype(bool)
    segments = []
    in_segment = False
    start_idx = None
    prev_idx = None
    for idx, flag in active_mask.items():
        if flag and not in_segment:
            start_idx = idx
            in_segment = True
        if not flag and in_segment:
            segments.append((start_idx, prev_idx))
            in_segment = False
        prev_idx = idx
    if in_segment:
        segments.append((start_idx, prev_idx))

    segment_effectiveness = []
    for start, end in segments:
        seg = valid.loc[start:end]
        touched_line = (seg['lateral_dist_px'] <= (CAR_WIDTH_PX/2)).any()
        segment_effectiveness.append(0.0 if touched_line else 100.0)

    num_segments = len(segment_effectiveness)
    successful_segments = sum(1 for v in segment_effectiveness if v == 100.0)
    pct_in_lane = np.mean(segment_effectiveness) if segment_effectiveness else np.nan
    
    # Departures during active periods (frame-level) for reference
    in_lane_active = active['lateral_dist_px'] > (CAR_WIDTH_PX/2)
    lane_changes = in_lane_active.astype(int).diff().fillna(0)
    departures = (lane_changes == -1).sum()
    
    # Average lateral error (distance from lane boundary)
    avg_lateral_error_px = active['lateral_dist_px'].mean()
    max_lateral_error_px = active['lateral_dist_px'].max()
    rms_lateral_error_px = np.sqrt((active['lateral_dist_px'] ** 2).mean())
    
    avg_lateral_error_m = active['lateral_dist_m'].mean()
    max_lateral_error_m = active['lateral_dist_m'].max()
    rms_lateral_error_m = np.sqrt((active['lateral_dist_m'] ** 2).mean())
    
    # Steering and speed stats
    avg_steering_rad = active['steering_rad'].mean()
    max_steering_rad = active['steering_rad'].abs().max()
    avg_speed_pps = active['speed_pps'].mean()
    avg_speed_mps = active['speed_mps'].mean()
    
    # LTA activation percentage over all valid frames
    lta_pct = (valid['lta_active'].sum() / len(valid)) * 100.0
    
    return {
        'total_frames': len(active),
        'duration_s': active['time_s'].max() - active['time_s'].min(),
        'pct_in_lane': pct_in_lane,
        'num_active_segments': num_segments,
        'successful_segments': successful_segments,
        'num_departures': departures,
        'avg_lateral_error_px': avg_lateral_error_px,
        'max_lateral_error_px': max_lateral_error_px,
        'rms_lateral_error_px': rms_lateral_error_px,
        'avg_lateral_error_m': avg_lateral_error_m,
        'max_lateral_error_m': max_lateral_error_m,
        'rms_lateral_error_m': rms_lateral_error_m,
        'avg_steering_rad': avg_steering_rad,
        'max_steering_rad': max_steering_rad,
        'avg_speed_pps': avg_speed_pps,
        'avg_speed_mps': avg_speed_mps,
        'lta_active_pct': lta_pct,
    }


def print_metrics(metrics, test_name="Test"):
    """Print metrics in a readable format."""
    print(f"\n{'='*60}")
    print(f"  {test_name}")
    print(f"{'='*60}")
    print(f"Duration: {metrics['duration_s']:.1f} s  ({metrics['total_frames']} frames)")
    print(f"\nLANE-KEEPING EFFECTIVENESS (LTA segments):")
    print(f"  Segment Success Rate: {metrics['pct_in_lane']:.1f}%")
    print(f"  Successful Segments:  {metrics.get('successful_segments', 0)} / {metrics.get('num_active_segments', 0)}")
    print(f"  Departures (frames):  {metrics['num_departures']}")
    print(f"\nLATERAL ERROR:")
    print(f"  Average (m):        {metrics['avg_lateral_error_m']:.3f} m")
    print(f"  RMS (m):            {metrics['rms_lateral_error_m']:.3f} m")
    print(f"  Max (m):            {metrics['max_lateral_error_m']:.3f} m")
    print(f"  Average (px):       {metrics['avg_lateral_error_px']:.1f} px")
    print(f"  RMS (px):           {metrics['rms_lateral_error_px']:.1f} px")
    print(f"  Max (px):           {metrics['max_lateral_error_px']:.1f} px")
    print(f"\nSPEED & STEERING:")
    print(f"  Average Speed:      {metrics['avg_speed_mps']:.2f} m/s ({metrics['avg_speed_pps']:.0f} px/s)")
    print(f"  Avg Steering Angle: {np.degrees(metrics['avg_steering_rad']):.1f}°")
    print(f"  Max Steering Angle: {np.degrees(metrics['max_steering_rad']):.1f}°")
    print(f"\nCONTROLLER ACTIVITY:")
    print(f"  LTA Active:         {metrics['lta_active_pct']:.1f}%")
    print(f"{'='*60}\n")


def plot_metrics(df, test_name="LTA Test"):
    """Create multi-panel plot of key metrics over time."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10))
    fig.suptitle(f"LTA Controller Performance: {test_name}", fontsize=14, fontweight='bold')
    
    # Lateral distance
    valid = df[df['lateral_dist_m'].notna()]
    axes[0].plot(valid['time_s'], valid['lateral_dist_m'], 'b-', linewidth=1.5, label='Lateral Distance (m)')
    axes[0].axhline(y=40/PPM, color='r', linestyle='--', alpha=0.7, label='LTA Threshold')
    axes[0].axhline(y=(CAR_WIDTH_M/2), color='orange', linestyle='-.', alpha=0.8, label='Lane Crossing (car half-width)')
    axes[0].set_ylabel('Distance (m)')
    axes[0].set_title('Lateral Distance from Lane')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Speed
    axes[1].plot(df['time_s'], df['speed_mps'], 'g-', linewidth=1.5, label='Speed')
    axes[1].set_ylabel('Speed (m/s)')
    axes[1].set_title('Vehicle Speed')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Steering angle
    axes[2].plot(df['time_s'], np.degrees(df['steering_rad']), 'm-', linewidth=1.5, label='Steering Angle')
    axes[2].set_ylabel('Steering Angle (°)')
    axes[2].set_title('Steering Angle')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # LTA active indicator
    axes[3].fill_between(df['time_s'], 0, df['lta_active'].astype(int), alpha=0.5, label='LTA Active')
    axes[3].set_ylabel('LTA Active')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('LTA Activation')
    axes[3].set_ylim([0, 1.1])
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def plot_aggregate_metrics(metrics_list, test_names):
    """
    Create a summary plot of % in lane across all tests.
    
    metrics_list: list of dicts from compute_lane_keeping_metrics()
    test_names: list of test name strings
    """
    pct_in_lane_vals = [m['pct_in_lane'] for m in metrics_list]
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Bar plot
    colors = plt.cm.RdYlGn(np.linspace(0.3, 0.9, len(pct_in_lane_vals)))
    bars = ax.bar(range(len(pct_in_lane_vals)), pct_in_lane_vals, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
    
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars, pct_in_lane_vals)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{val:.1f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Summary statistics
    mean_val = np.mean(pct_in_lane_vals)
    std_val = np.std(pct_in_lane_vals)
    min_val = np.min(pct_in_lane_vals)
    max_val = np.max(pct_in_lane_vals)
    
    ax.axhline(y=mean_val, color='blue', linestyle='--', linewidth=2, label=f'Mean: {mean_val:.1f}%')
    ax.fill_between(range(-1, len(pct_in_lane_vals)+1), 
                     mean_val - std_val, mean_val + std_val, 
                     alpha=0.2, color='blue', label=f'±1 Std Dev: {std_val:.1f}%')
    ax.axhline(y=95, color='green', linestyle=':', linewidth=2, alpha=0.7, label='Target (95%)')
    
    ax.set_xlabel('Test', fontsize=12, fontweight='bold')
    ax.set_ylabel('% Time In Lane', fontsize=12, fontweight='bold')
    ax.set_title('LTA Lane-Keeping Effectiveness Across Tests', fontsize=14, fontweight='bold')
    ax.set_xticks(range(len(test_names)))
    ax.set_xticklabels(test_names, rotation=45, ha='right')
    ax.set_ylim([0, 105])
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    return fig


def print_aggregate_summary(metrics_list, test_names):
    """Print summary of all tests."""
    pct_vals = [m['pct_in_lane'] for m in metrics_list]
    
    print(f"\n{'='*70}")
    print("  AGGREGATE LTA EFFECTIVENESS SUMMARY (All Tests)")
    print(f"{'='*70}")
    print(f"\n{'Test Name':<30} {'% In Lane':>15} {'Departures':>12}")
    print(f"{'-'*70}")
    
    for name, metrics in zip(test_names, metrics_list):
        print(f"{name:<30} {metrics['pct_in_lane']:>14.1f}% {metrics['num_departures']:>12}")
    
    print(f"{'-'*70}")
    print(f"{'MEAN':<30} {np.mean(pct_vals):>14.1f}%")
    print(f"{'STD DEV':<30} {np.std(pct_vals):>14.1f}%")
    print(f"{'MIN':<30} {np.min(pct_vals):>14.1f}%")
    print(f"{'MAX':<30} {np.max(pct_vals):>14.1f}%")
    print(f"{'='*70}\n")


def main():
    # Find all log files
    all_logs = sorted(Path('.').glob('lta_test_*.csv'), key=lambda p: p.stat().st_mtime, reverse=True)
    
    if len(sys.argv) > 1:
        # Single log mode
        log_path = sys.argv[1]
        df = load_log(log_path)
        if df is None:
            sys.exit(1)
        
        metrics = compute_lane_keeping_metrics(df, lane_threshold_px=40.0)
        print_metrics(metrics, test_name=str(log_path))
        
        fig = plot_metrics(df, test_name=str(log_path))
        plot_file = str(log_path).replace('.csv', '_analysis.png')
        fig.savefig(plot_file, dpi=100, bbox_inches='tight')
        print(f"Saved plot to: {plot_file}")
        
        plt.show()
    
    elif all_logs:
        # Multi-log aggregate mode
        print(f"Found {len(all_logs)} log files. Computing aggregate statistics...\n")
        
        metrics_list = []
        test_names = []
        dfs = []
        
        for log_path in all_logs:
            df = load_log(log_path)
            if df is not None:
                metrics = compute_lane_keeping_metrics(df, lane_threshold_px=40.0)
                metrics_list.append(metrics)
                dfs.append(df)
                # Extract short test name from filename
                test_name = str(log_path.stem).replace('lta_test_', '')
                test_names.append(test_name)
        
        if metrics_list:
            # Print aggregate summary
            print_aggregate_summary(metrics_list, test_names)
            
            # Print detailed metrics for last (most recent) test
            print_metrics(metrics_list[0], test_name=f"LAST TEST: {test_names[0]}")
            
            # Plot last test details
            fig_last = plot_metrics(dfs[0], test_name=test_names[0])
            last_plot_file = f'lta_test_{test_names[0]}_analysis.png'
            fig_last.savefig(last_plot_file, dpi=100, bbox_inches='tight')
            print(f"Saved last test plot to: {last_plot_file}")
            
            # Create aggregate plot
            fig_agg = plot_aggregate_metrics(metrics_list, test_names)
            agg_plot_file = 'lta_aggregate_summary.png'
            fig_agg.savefig(agg_plot_file, dpi=100, bbox_inches='tight')
            print(f"Saved aggregate plot to: {agg_plot_file}")
            
            plt.show()
    else:
        print("Usage: python analysis_lta.py [log_file.csv]")
        print("  - With argument: analyze single log file")
        print("  - Without argument: analyze ALL lta_test_*.csv files in current directory")
        sys.exit(1)


if __name__ == '__main__':
    main()
