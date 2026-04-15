def print_report(report):
    print("\n===== FINAL REPORT =====")
    print(f"Boxes detected: {len(report.boxes)}")
    for c, b in report.boxes.items():
        print(f"{c}: {b.pose}, detected={b.detected}")
    print(f"Blocks found: {report.blocks_found}")
    print(f"Interest points: total={report.interest_points_total}, visited={report.interest_points_visited}, skipped={report.interest_points_skipped}")
    print(f"Docking: {report.docking_success}/{report.docking_attempts}")
    print(f"Pick-Place success: {report.pick_place_success}")
    print("========================\n")
