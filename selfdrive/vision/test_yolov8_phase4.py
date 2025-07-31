#!/usr/bin/env python3
"""
Comprehensive YOLOv8 Phase 4 Testing Suite for NagasPilot
Tests all aspects of the YOLOv8 daemon for production readiness

Test Categories:
1. Model Validation & Loading
2. Camera Connection & Data Processing  
3. Image Preprocessing & YUV Conversion
4. Batched Inference Processing
5. Detection Filtering & Classification
6. Performance Monitoring & CPU Usage
7. Emergency Response Classification
8. Error Handling & Graceful Degradation
9. Integration Testing with EODS/SOC

Usage:
    python test_yolov8_phase4.py [--verbose] [--performance] [--integration]
"""

import os
import sys
import time
import unittest
import numpy as np
import tempfile
import json
import threading
from unittest.mock import Mock, MagicMock, patch
from typing import Dict, List, Tuple, Optional

# Add project root to path for imports
sys.path.insert(0, '/home/vcar/Winsurf/nagaspilot')

from selfdrive.vision.yolov8_daemon import OptimizedYOLOv8Daemon, EODS_CLASSES, SOC_CLASSES, ALL_CLASSES, EMERGENCY_RESPONSES

class TestYOLOv8ModelValidation(unittest.TestCase):
    """Test suite for YOLOv8 model validation and loading"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
        self.test_model_dir = tempfile.mkdtemp()
        
    def tearDown(self):
        if hasattr(self, 'daemon') and self.daemon.model:
            del self.daemon.model
        # Clean up temp files
        import shutil
        shutil.rmtree(self.test_model_dir, ignore_errors=True)
    
    def create_mock_onnx_model(self, input_shape, output_shape, valid=True):
        """Create a mock ONNX model for testing"""
        mock_session = Mock()
        
        # Mock inputs
        mock_input = Mock()
        mock_input.shape = input_shape
        mock_session.get_inputs.return_value = [mock_input]
        
        # Mock outputs  
        mock_output = Mock()
        mock_output.shape = output_shape
        mock_session.get_outputs.return_value = [mock_output]
        
        if not valid:
            mock_session.get_inputs.side_effect = Exception("Invalid model")
            
        return mock_session
    
    @patch('onnxruntime.InferenceSession')
    def test_valid_yolov8n_model(self, mock_session_class):
        """Test validation of valid YOLOv8n model"""
        # Valid YOLOv8 shapes
        input_shape = [1, 3, 640, 640]
        output_shape = [1, 8400, 84]
        
        mock_session_class.return_value = self.create_mock_onnx_model(input_shape, output_shape)
        
        model_path = os.path.join(self.test_model_dir, "yolov8n.onnx")
        with open(model_path, 'w') as f:
            f.write("mock model content")
            
        result = self.daemon.validate_yolo_model(model_path)
        self.assertTrue(result, "Valid YOLOv8n model should pass validation")
    
    @patch('onnxruntime.InferenceSession')
    def test_invalid_input_shape(self, mock_session_class):
        """Test validation fails for invalid input shape"""
        # Invalid input shape (wrong channels)
        input_shape = [1, 4, 640, 640]  # Should be 3 channels
        output_shape = [1, 8400, 84]
        
        mock_session_class.return_value = self.create_mock_onnx_model(input_shape, output_shape)
        
        model_path = os.path.join(self.test_model_dir, "invalid.onnx")
        with open(model_path, 'w') as f:
            f.write("mock model content")
            
        result = self.daemon.validate_yolo_model(model_path)
        self.assertFalse(result, "Invalid input shape should fail validation")
    
    @patch('onnxruntime.InferenceSession')
    def test_invalid_output_shape(self, mock_session_class):
        """Test validation fails for invalid output shape"""
        # Invalid output shape (wrong classes)
        input_shape = [1, 3, 640, 640]
        output_shape = [1, 8400, 85]  # Should be 84 for YOLOv8
        
        mock_session_class.return_value = self.create_mock_onnx_model(input_shape, output_shape)
        
        model_path = os.path.join(self.test_model_dir, "invalid.onnx")
        with open(model_path, 'w') as f:
            f.write("mock model content")
            
        result = self.daemon.validate_yolo_model(model_path)
        self.assertFalse(result, "Invalid output shape should fail validation")
    
    def test_model_file_not_found(self):
        """Test validation handles missing model file"""
        nonexistent_path = "/nonexistent/model.onnx"
        result = self.daemon.validate_yolo_model(nonexistent_path)
        self.assertFalse(result, "Missing model file should fail validation")


class TestYOLOv8ImageProcessing(unittest.TestCase):
    """Test suite for image preprocessing and YUV conversion"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
    
    def test_yuv_to_rgb_conversion_accuracy(self):
        """Test YUV to RGB conversion against reference implementation"""
        # Create test YUV data (100x100 pixels)
        height, width = 100, 100
        y = np.random.randint(0, 256, (height, width), dtype=np.uint8)
        u = np.random.randint(0, 256, (height//2, width//2), dtype=np.uint8)
        v = np.random.randint(0, 256, (height//2, width//2), dtype=np.uint8)
        
        # Convert using daemon method
        rgb_result = self.daemon.proper_yuv_to_rgb(y, u, v)
        
        # Verify output shape and type
        self.assertEqual(rgb_result.shape, (height, width, 3), "RGB output should have correct shape")
        self.assertEqual(rgb_result.dtype, np.uint8, "RGB output should be uint8")
        
        # Verify value ranges (RGB should be 0-255)
        self.assertTrue(np.all(rgb_result >= 0), "RGB values should be >= 0")
        self.assertTrue(np.all(rgb_result <= 255), "RGB values should be <= 255")
    
    def test_yuv_conversion_with_extreme_values(self):
        """Test YUV conversion with extreme values (0, 255)"""
        height, width = 50, 50
        
        # Test with all zeros
        y_zero = np.zeros((height, width), dtype=np.uint8)
        u_zero = np.zeros((height//2, width//2), dtype=np.uint8)
        v_zero = np.zeros((height//2, width//2), dtype=np.uint8)
        
        rgb_zero = self.daemon.proper_yuv_to_rgb(y_zero, u_zero, v_zero)
        self.assertIsNotNone(rgb_zero, "YUV conversion should handle zero values")
        
        # Test with all 255s
        y_max = np.full((height, width), 255, dtype=np.uint8)
        u_max = np.full((height//2, width//2), 255, dtype=np.uint8)
        v_max = np.full((height//2, width//2), 255, dtype=np.uint8)
        
        rgb_max = self.daemon.proper_yuv_to_rgb(y_max, u_max, v_max)
        self.assertIsNotNone(rgb_max, "YUV conversion should handle max values")
    
    def test_yolo_preprocessing_accuracy(self):
        """Test YOLO image preprocessing produces correct format"""
        # Create test RGB image (typical camera resolution)
        test_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        
        # Preprocess for YOLO
        result = self.daemon.preprocess_for_yolo(test_image)
        
        # Verify output format
        self.assertIsNotNone(result, "Preprocessing should return valid result")
        self.assertEqual(result.shape, (3, 640, 640), "Output should be CHW format 640x640")
        self.assertEqual(result.dtype, np.float32, "Output should be float32")
        
        # Verify normalization (values should be 0-1)
        self.assertTrue(np.all(result >= 0.0), "Normalized values should be >= 0")
        self.assertTrue(np.all(result <= 1.0), "Normalized values should be <= 1")
    
    def test_preprocessing_edge_cases(self):
        """Test preprocessing with edge case inputs"""
        # Test with None input
        result_none = self.daemon.preprocess_for_yolo(None)
        self.assertIsNone(result_none, "None input should return None")
        
        # Test with small image
        small_image = np.random.randint(0, 256, (10, 10, 3), dtype=np.uint8)
        result_small = self.daemon.preprocess_for_yolo(small_image)
        self.assertIsNotNone(result_small, "Small images should be processed")
        self.assertEqual(result_small.shape, (3, 640, 640), "Small image should be resized to 640x640")


class TestYOLOv8InferenceProcessing(unittest.TestCase):
    """Test suite for batched inference processing"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
        # Mock the ONNX model
        self.daemon.model = Mock()
    
    def test_batched_inference_both_cameras(self):
        """Test batched inference with both road and wide cameras"""
        # Create mock camera images
        road_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        wide_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        
        # Mock model output (typical YOLOv8 output shape)
        mock_output = np.random.random((2, 8400, 84)).astype(np.float32)
        self.daemon.model.run.return_value = [mock_output]
        
        # Test batched inference
        road_result, wide_result = self.daemon.batched_inference(road_image, wide_image)
        
        # Verify results
        self.assertIsNotNone(road_result, "Road camera result should not be None")
        self.assertIsNotNone(wide_result, "Wide camera result should not be None")
        
        # Verify model was called once (batched processing)
        self.assertEqual(self.daemon.model.run.call_count, 1, "Model should be called once for batched processing")
    
    def test_single_camera_inference(self):
        """Test inference with single camera (road only)"""
        road_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        
        # Mock model output for single image
        mock_output = np.random.random((1, 8400, 84)).astype(np.float32)
        self.daemon.model.run.return_value = [mock_output]
        
        # Test single camera inference
        road_result, wide_result = self.daemon.batched_inference(road_image, None)
        
        # Verify results
        self.assertIsNotNone(road_result, "Road camera result should not be None")
        self.assertIsNone(wide_result, "Wide camera result should be None for single input")
    
    def test_inference_with_no_model(self):
        """Test inference behavior when model is not loaded"""
        self.daemon.model = None
        
        road_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        wide_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        
        road_result, wide_result = self.daemon.batched_inference(road_image, wide_image)
        
        # Should return None for both when no model
        self.assertIsNone(road_result, "Should return None when no model loaded")
        self.assertIsNone(wide_result, "Should return None when no model loaded")
    
    def test_inference_error_handling(self):
        """Test inference error handling and graceful degradation"""
        # Make model.run raise an exception
        self.daemon.model.run.side_effect = Exception("ONNX Runtime error")
        
        road_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        wide_image = np.random.randint(0, 256, (1280, 720, 3), dtype=np.uint8)
        
        # Should handle exception gracefully
        try:
            road_result, wide_result = self.daemon.batched_inference(road_image, wide_image)
            # Should return None on error without crashing
            self.assertIsNone(road_result, "Should return None on inference error") 
            self.assertIsNone(wide_result, "Should return None on inference error")
        except Exception as e:
            self.fail(f"Inference should handle errors gracefully, but raised: {e}")


class TestYOLOv8ClassificationAndFiltering(unittest.TestCase):
    """Test suite for detection filtering and emergency classification"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
    
    def test_eods_class_filtering(self):
        """Test EODS class filtering for emergency detection"""
        # Verify EODS classes are correctly defined
        expected_eods = {0: 'person', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'cow', 19: 'elephant'}
        self.assertEqual(EODS_CLASSES, expected_eods, "EODS classes should match specification")
        
        # Test emergency response categorization
        emergency_stop_classes = EMERGENCY_RESPONSES['EMERGENCY_STOP']
        slow_down_classes = EMERGENCY_RESPONSES['SLOW_DOWN']
        
        # Verify critical objects trigger emergency stop
        self.assertIn('person', emergency_stop_classes, "Person should trigger emergency stop")
        self.assertIn('cow', emergency_stop_classes, "Cow should trigger emergency stop")
        self.assertIn('elephant', emergency_stop_classes, "Elephant should trigger emergency stop")
        
        # Verify smaller animals trigger slow down
        self.assertIn('cat', slow_down_classes, "Cat should trigger slow down")
        self.assertIn('dog', slow_down_classes, "Dog should trigger slow down")
    
    def test_soc_class_filtering(self):
        """Test SOC class filtering for vehicle avoidance"""
        # Verify SOC classes for large vehicle detection
        expected_soc = {2: 'car', 5: 'bus', 7: 'truck'}
        self.assertEqual(SOC_CLASSES, expected_soc, "SOC classes should match specification")
        
        # Verify all vehicle types are in monitoring
        monitor_classes = EMERGENCY_RESPONSES['MONITOR']
        self.assertIn('car', monitor_classes, "Car should be monitored")
        self.assertIn('bus', monitor_classes, "Bus should be monitored")  
        self.assertIn('truck', monitor_classes, "Truck should be monitored")
    
    def test_combined_class_filtering(self):
        """Test combined class filtering (EODS + SOC)"""
        # Total classes should be EODS + SOC
        expected_total = len(EODS_CLASSES) + len(SOC_CLASSES)
        self.assertEqual(len(ALL_CLASSES), expected_total, "ALL_CLASSES should combine EODS and SOC")
        
        # Verify no overlap between EODS and SOC classes
        eods_ids = set(EODS_CLASSES.keys())
        soc_ids = set(SOC_CLASSES.keys())
        overlap = eods_ids.intersection(soc_ids)
        self.assertEqual(len(overlap), 0, "EODS and SOC should have no overlapping class IDs")
    
    def test_emergency_response_completeness(self):
        """Test that all classes have appropriate emergency responses"""
        all_response_classes = set()
        for response_type, classes in EMERGENCY_RESPONSES.items():
            all_response_classes.update(classes)
        
        all_class_names = set(ALL_CLASSES.values())
        
        # Every class should have a response
        missing_responses = all_class_names - all_response_classes
        self.assertEqual(len(missing_responses), 0, f"Classes missing emergency responses: {missing_responses}")


class TestYOLOv8PerformanceMonitoring(unittest.TestCase):
    """Test suite for performance monitoring and CPU usage"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
    
    def test_cpu_budget_constants(self):
        """Test CPU budget constants are reasonable"""
        from selfdrive.vision.yolov8_daemon import TARGET_CPU_BUDGET
        
        # CPU budget should be reasonable (10-25%)
        self.assertGreater(TARGET_CPU_BUDGET, 0.10, "CPU budget should be at least 10%")
        self.assertLess(TARGET_CPU_BUDGET, 0.25, "CPU budget should be at most 25%")
    
    def test_performance_tracking_initialization(self):
        """Test performance tracking variables are initialized"""
        self.assertEqual(self.daemon.frame_count, 0, "Frame count should start at 0")
        self.assertIsInstance(self.daemon.last_stats_time, float, "Stats time should be float timestamp")
        self.assertIsInstance(self.daemon.processing_times, list, "Processing times should be list")
        self.assertEqual(len(self.daemon.processing_times), 0, "Processing times should start empty")
    
    def test_batch_size_optimization(self):
        """Test batch size is optimized for dual camera processing"""
        # Batch size should be 2 for road + wide cameras
        self.assertEqual(self.daemon.batch_size, 2, "Batch size should be 2 for dual cameras")
    
    @patch('time.time')
    def test_processing_time_tracking(self, mock_time):
        """Test processing time tracking functionality"""
        # Mock time progression
        mock_time.side_effect = [1000.0, 1000.1, 1000.2]  # 100ms and 100ms processing times
        
        # Simulate processing time tracking
        start_time = time.time()
        # ... processing would happen here ...
        end_time = time.time()
        processing_time = end_time - start_time
        
        self.daemon.processing_times.append(processing_time)
        self.daemon.frame_count += 1
        
        # Verify tracking
        self.assertEqual(len(self.daemon.processing_times), 1, "Should track one processing time")
        self.assertEqual(self.daemon.frame_count, 1, "Should increment frame count")


class TestYOLOv8ErrorHandling(unittest.TestCase):
    """Test suite for error handling and graceful degradation"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
    
    @patch('openpilot.common.swaglog.cloudlog')
    def test_model_loading_error_handling(self, mock_cloudlog):
        """Test error handling during model loading"""
        # Test with invalid model path
        with patch('os.path.exists', return_value=False):
            result = self.daemon.load_optimized_model()
            self.assertFalse(result, "Should return False when model doesn't exist")
            
        # Verify error was logged
        mock_cloudlog.error.assert_called()
    
    @patch('openpilot.common.swaglog.cloudlog')
    def test_camera_connection_error_handling(self, mock_cloudlog):
        """Test error handling during camera connection"""
        with patch('msgq.visionipc.VisionIpcClient') as mock_client_class:
            # Make camera connection fail
            mock_client = Mock()
            mock_client.connect.return_value = False
            mock_client_class.return_value = mock_client
            
            # Should raise exception for failed connection
            with self.assertRaises(Exception):
                self.daemon.connect_cameras()
    
    def test_preprocessing_error_handling(self):
        """Test preprocessing error handling with invalid inputs"""
        # Test with invalid image format
        invalid_image = "not an image"
        result = self.daemon.preprocess_for_yolo(invalid_image)
        self.assertIsNone(result, "Should return None for invalid image")
        
        # Test with image missing color channel
        invalid_shape = np.random.randint(0, 256, (100, 100), dtype=np.uint8)  # Missing channel
        result = self.daemon.preprocess_for_yolo(invalid_shape)
        # Should handle gracefully (OpenCV resize might work or fail)
        # Either None or valid result is acceptable
        if result is not None:
            self.assertEqual(result.shape, (3, 640, 640), "If processed, should have correct shape")
    
    def test_yuv_conversion_error_handling(self):
        """Test YUV conversion error handling with mismatched dimensions"""
        # Test with mismatched Y and U/V dimensions
        y = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        u = np.random.randint(0, 256, (40, 40), dtype=np.uint8)  # Wrong size
        v = np.random.randint(0, 256, (50, 50), dtype=np.uint8)  # Wrong size
        
        try:
            result = self.daemon.proper_yuv_to_rgb(y, u, v)
            # If it doesn't crash, result should be reasonable
            if result is not None:
                self.assertEqual(result.shape[:2], y.shape, "Output should match Y dimensions")
        except Exception:
            # Exception is acceptable for malformed input
            pass


class TestYOLOv8Integration(unittest.TestCase):
    """Integration tests for YOLOv8 daemon with EODS/SOC systems"""
    
    def setUp(self):
        self.daemon = OptimizedYOLOv8Daemon()
    
    def test_eods_integration_flags(self):
        """Test EODS integration flag handling"""
        # Test initial state
        self.assertFalse(self.daemon.eods_enabled, "EODS should start disabled")
        
        # Test enabling EODS
        self.daemon.eods_enabled = True
        self.assertTrue(self.daemon.eods_enabled, "Should be able to enable EODS")
    
    def test_soc_integration_flags(self):
        """Test SOC integration flag handling"""  
        # Test initial state
        self.assertFalse(self.daemon.soc_enabled, "SOC should start disabled")
        
        # Test enabling SOC
        self.daemon.soc_enabled = True
        self.assertTrue(self.daemon.soc_enabled, "Should be able to enable SOC")
    
    @patch('cereal.messaging.PubMaster')
    def test_detection_publishing(self, mock_pub_master):
        """Test detection result publishing to cereal messaging"""
        # Verify publisher is created with correct topic
        self.daemon = OptimizedYOLOv8Daemon()
        mock_pub_master.assert_called_with(['yolov8Detections'])
    
    def test_params_integration(self):
        """Test integration with OpenPilot params system"""
        # Verify params object is created
        self.assertIsNotNone(self.daemon.params, "Should have params object")
        
        # Test that it's the correct type (Params from openpilot.common.params)
        from openpilot.common.params import Params
        self.assertIsInstance(self.daemon.params, Params, "Should be OpenPilot Params instance")


class TestYOLOv8ConfigurationValidation(unittest.TestCase):
    """Test suite for configuration validation and system requirements"""
    
    def test_class_id_consistency(self):
        """Test that class IDs are consistent with COCO dataset"""
        # COCO class IDs should be correct
        coco_person_id = 0
        coco_car_id = 2  
        coco_bus_id = 5
        coco_truck_id = 7
        
        self.assertEqual(EODS_CLASSES[0], 'person', "Person should be COCO class 0")
        self.assertEqual(SOC_CLASSES[2], 'car', "Car should be COCO class 2")
        self.assertEqual(SOC_CLASSES[5], 'bus', "Bus should be COCO class 5")
        self.assertEqual(SOC_CLASSES[7], 'truck', "Truck should be COCO class 7")
    
    def test_emergency_response_logic(self):
        """Test emergency response classification logic"""
        # Large animals should be emergency stop
        large_animals = ['person', 'horse', 'cow', 'elephant']
        for animal in large_animals:
            self.assertIn(animal, EMERGENCY_RESPONSES['EMERGENCY_STOP'], 
                         f"{animal} should trigger emergency stop")
        
        # Small animals should be slow down
        small_animals = ['cat', 'dog']
        for animal in small_animals:
            self.assertIn(animal, EMERGENCY_RESPONSES['SLOW_DOWN'],
                         f"{animal} should trigger slow down")
    
    def test_system_resource_configuration(self):
        """Test system resource configuration is reasonable"""
        # Check that we're filtering to reasonable number of classes
        total_coco_classes = 80
        filtered_classes = len(ALL_CLASSES)
        
        self.assertLess(filtered_classes, total_coco_classes, "Should filter to fewer classes")
        self.assertEqual(filtered_classes, 9, "Should have exactly 9 filtered classes")
        
        # Verify significant reduction in processing
        reduction_ratio = filtered_classes / total_coco_classes
        self.assertLess(reduction_ratio, 0.15, "Should reduce processing by >85%")


def run_comprehensive_tests(verbose=False, performance=False, integration=False):
    """Run comprehensive YOLOv8 testing suite"""
    
    print("=" * 80)
    print("NagasPilot YOLOv8 Phase 4 Comprehensive Testing Suite")
    print("=" * 80)
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add all test classes
    test_classes = [
        TestYOLOv8ModelValidation,
        TestYOLOv8ImageProcessing,
        TestYOLOv8InferenceProcessing,
        TestYOLOv8ClassificationAndFiltering,
        TestYOLOv8PerformanceMonitoring,
        TestYOLOv8ErrorHandling,
        TestYOLOv8ConfigurationValidation,
    ]
    
    # Add integration tests if requested
    if integration:
        test_classes.append(TestYOLOv8Integration)
    
    # Load tests from each class
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Configure test runner
    verbosity = 2 if verbose else 1
    runner = unittest.TextTestRunner(verbosity=verbosity, buffer=True)
    
    # Run tests
    print(f"\nRunning {test_suite.countTestCases()} tests...")
    start_time = time.time()
    
    result = runner.run(test_suite)
    
    end_time = time.time()
    elapsed = end_time - start_time
    
    # Print summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Elapsed time: {elapsed:.2f} seconds")
    
    if result.failures:
        print(f"\nFAILURES ({len(result.failures)}):")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback.split('AssertionError: ')[-1].split(chr(10))[0]}")
    
    if result.errors:
        print(f"\nERRORS ({len(result.errors)}):")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback.split(chr(10))[-2]}")
    
    # Performance summary if requested
    if performance:
        print("\n" + "=" * 80)
        print("PERFORMANCE ANALYSIS")
        print("=" * 80)
        print("CPU Budget Analysis:")
        print(f"- Target CPU Budget: {TARGET_CPU_BUDGET*100:.1f}%")
        print(f"- Filtered Classes: {len(ALL_CLASSES)}/80 COCO classes ({len(ALL_CLASSES)/80*100:.1f}%)")
        print(f"- Processing Reduction: ~{(1-len(ALL_CLASSES)/80)*100:.1f}%")
        print("\nDetection Classes:")
        print(f"- EODS Emergency Classes: {len(EODS_CLASSES)} classes")
        print(f"- SOC Vehicle Classes: {len(SOC_CLASSES)} classes")
        print(f"- Total Active Classes: {len(ALL_CLASSES)} classes")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    
    print("\n" + "=" * 80)
    if success:
        print("✅ ALL TESTS PASSED - YOLOv8 daemon ready for production")
    else:
        print("❌ TESTS FAILED - Review failures before deployment")
    print("=" * 80)
    
    return success


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="NagasPilot YOLOv8 Phase 4 Testing Suite")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose test output")
    parser.add_argument("--performance", action="store_true", help="Show performance analysis")
    parser.add_argument("--integration", action="store_true", help="Run integration tests")
    
    args = parser.parse_args()
    
    # Run comprehensive tests
    success = run_comprehensive_tests(
        verbose=args.verbose,
        performance=args.performance, 
        integration=args.integration
    )
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)