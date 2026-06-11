#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sim_connection.py — Verifica que la conexión Sim2Real funcione.

Ejecutar DESPUÉS de que Unity esté escuchando en 127.0.0.1:5005:
  python test_sim_connection.py

Resultado esperado: "ALL TESTS PASSED ✓"
"""

import time
import sys

def test_connection():
    print("=" * 70)
    print("SIM2REAL CONNECTION TEST — Phase 1 Validation")
    print("=" * 70)
    
    try:
        # Test 1: Import
        print("\n[TEST 1/6] Importing simulator client...")
        try:
            from sim_hardware_mocks import SimulatorClient
            print("  ✓ sim_hardware_mocks imported successfully")
        except ImportError as e:
            print(f"  ✗ FAILED: {e}")
            print("  Hint: Run from TMR2026/ directory: cd TMR2026 && python test_sim_connection.py")
            return False
        
        # Test 2: Connection
        print("\n[TEST 2/6] Connecting to Unity simulator (127.0.0.1:5005)...")
        try:
            sim = SimulatorClient(host='127.0.0.1', port=5005, timeout=5.0)
            print("  ✓ Connected to simulator!")
        except ConnectionRefusedError:
            print("  ✗ FAILED: Connection refused")
            print("  → Is Unity running and listening on 127.0.0.1:5005?")
            return False
        except Exception as e:
            print(f"  ✗ FAILED: {e}")
            return False
        
        # Test 3: Motor control
        print("\n[TEST 3/6] Testing motor (sending 25% PWM)...")
        try:
            sim.motor.set_speed(25.0)
            time.sleep(0.2)
            if sim.motor.current_duty == 25.0:
                print(f"  ✓ Motor set to 25.0% (current: {sim.motor.current_duty}%)")
            else:
                print(f"  ⚠ Motor set but value mismatch: {sim.motor.current_duty}%")
        except Exception as e:
            print(f"  ✗ FAILED: {e}")
            return False
        
        # Test 4: Steering control
        print("\n[TEST 4/6] Testing steering (sending 75° left)...")
        try:
            sim.steering.set_angle(75.0)
            time.sleep(0.2)
            if abs(sim.steering.current_angle - 75.0) < 0.1:
                print(f"  ✓ Steering set to 75.0° (current: {sim.steering.current_angle:.1f}°)")
            else:
                print(f"  ⚠ Steering set but value mismatch: {sim.steering.current_angle}°")
        except Exception as e:
            print(f"  ✗ FAILED: {e}")
            return False
        
        # Test 5: Sensors (10 seconds)
        print("\n[TEST 5/6] Reading sensors for 10 seconds...")
        start = time.time()
        frame_count = 0
        tof_count = 0
        tof_readings = []
        
        while time.time() - start < 10.0:
            # Read ToF
            if sim.distance.front_mm is not None:
                tof_count += 1
                tof_readings.append(sim.distance.front_mm)
            
            # Read camera
            frame = sim.camera.get_latest_frame()
            if frame is not None:
                frame_count += 1
                if frame_count == 1:
                    h, w = frame.shape[:2]
                    print(f"  ✓ First frame received: {h}×{w} pixels (BGR)")
            
            time.sleep(0.05)
        
        elapsed = time.time() - start
        print(f"  ✓ ToF readings: {tof_count} in {elapsed:.1f}s (expected ~50 @ 50 Hz)")
        print(f"  ✓ Camera frames: {frame_count} in {elapsed:.1f}s (expected ~30 @ 30 FPS)")
        
        if tof_count < 20:
            print(f"  ⚠ WARNING: ToF data sparse ({tof_count} readings). Check Unity sensor rate.")
        if frame_count < 10:
            print(f"  ⚠ WARNING: Camera frames sparse ({frame_count} frames). Check JPEG encoding.")
        
        if tof_readings:
            avg_dist = sum(tof_readings) / len(tof_readings)
            print(f"  ✓ ToF distance: {avg_dist:.0f} mm (avg of {len(tof_readings)} readings)")
        
        # Test 6: Brake
        print("\n[TEST 6/6] Testing brake...")
        try:
            sim.motor.brake()
            time.sleep(0.2)
            if sim.motor.current_duty == 0.0:
                print(f"  ✓ Motor braked successfully (duty = 0.0%)")
            else:
                print(f"  ⚠ Brake set but duty not zero: {sim.motor.current_duty}%")
        except Exception as e:
            print(f"  ✗ FAILED: {e}")
            return False
        
        # Cleanup
        sim.close()
        time.sleep(0.1)
        
        print("\n" + "=" * 70)
        print("✓ ALL TESTS PASSED")
        print("=" * 70)
        print("\nPhase 1 Sim2Real validation ready!")
        print("\nNext steps:")
        print("  1. Run vision debug:   python main_simulator.py --display")
        print("  2. Run autonomous:     python main_simulator.py")
        print("  3. Run validation:     See docs/PHASE1_VALIDATION.md")
        
        return True
        
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
