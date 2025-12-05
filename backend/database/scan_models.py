"""
Database Models for WeldVision X5 Scanning System

Stores scan results, measurements, and defect data
"""

from datetime import datetime
from sqlalchemy import Column, Integer, Float, String, DateTime, Boolean, ForeignKey, Text, Enum as SQLEnum
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
import enum
import json

Base = declarative_base()


class ScanStatus(str, enum.Enum):
    """Scan status enumeration"""
    PENDING = "pending"
    INITIALIZING = "initializing"
    IN_PROGRESS = "in_progress"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class DefectType(str, enum.Enum):
    """Types of defects that can be detected"""
    POROSITY = "porosity"
    GAP = "gap"
    SURFACE_IRREGULARITY = "surface_irregularity"
    UNDERCUT = "undercut"
    SPATTER = "spatter"
    UNKNOWN = "unknown"


class ScanConfiguration(Base):
    """Stores scan configuration parameters"""
    __tablename__ = 'scan_configurations'
    
    id = Column(Integer, primary_key=True)
    name = Column(String(100), nullable=False)
    
    # Grid parameters
    grid_spacing_mm = Column(Float, default=25.0)  # Distance between scan points
    grid_overlap_mm = Column(Float, default=10.0)  # Overlap between scan areas
    
    # Z-height parameters
    z_height_mm = Column(Float, default=10.0)  # Camera height above workpiece
    z_safe_height_mm = Column(Float, default=50.0)  # Safe height for moves
    
    # Capture parameters
    dwell_time_ms = Column(Integer, default=500)  # Time at each position before capture
    capture_count = Column(Integer, default=3)  # Frames to capture at each position
    
    # Printer parameters
    feedrate_xy_mm_min = Column(Float, default=3000.0)  # XY movement speed
    feedrate_z_mm_min = Column(Float, default=800.0)  # Z movement speed
    
    # Vision parameters
    rgb_brightness_threshold = Column(Float, default=100.0)
    rgb_saturation_threshold = Column(Float, default=50.0)
    depth_invalid_value = Column(Integer, default=0)
    
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    scans = relationship("Scan", back_populates="configuration")
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'id': self.id,
            'name': self.name,
            'grid_spacing_mm': self.grid_spacing_mm,
            'grid_overlap_mm': self.grid_overlap_mm,
            'z_height_mm': self.z_height_mm,
            'z_safe_height_mm': self.z_safe_height_mm,
            'dwell_time_ms': self.dwell_time_ms,
            'capture_count': self.capture_count,
            'feedrate_xy_mm_min': self.feedrate_xy_mm_min,
            'feedrate_z_mm_min': self.feedrate_z_mm_min,
            'rgb_brightness_threshold': self.rgb_brightness_threshold,
            'rgb_saturation_threshold': self.rgb_saturation_threshold,
            'depth_invalid_value': self.depth_invalid_value,
        }


class Scan(Base):
    """Main scan record"""
    __tablename__ = 'scans'
    
    id = Column(Integer, primary_key=True)
    scan_id = Column(String(50), unique=True, nullable=False)  # External scan ID
    
    # References
    student_id = Column(Integer, ForeignKey('students.id'), nullable=True)
    configuration_id = Column(Integer, ForeignKey('scan_configurations.id'), nullable=True)
    
    # Status
    status = Column(SQLEnum(ScanStatus), default=ScanStatus.PENDING)
    
    # Scan metadata
    workpiece_type = Column(String(100))  # Type of weld/workpiece
    workpiece_material = Column(String(100))  # Material being scanned
    operator_notes = Column(Text)  # Notes from operator
    
    # Grid parameters (stored for record)
    grid_x_min = Column(Float)
    grid_x_max = Column(Float)
    grid_y_min = Column(Float)
    grid_y_max = Column(Float)
    grid_spacing = Column(Float)
    
    # Scan statistics
    total_points = Column(Integer, default=0)  # Total scan points planned
    completed_points = Column(Integer, default=0)  # Completed points
    failed_points = Column(Integer, default=0)  # Failed captures
    
    # Processing results
    total_point_cloud_size = Column(Integer, default=0)  # Total 3D points
    average_measurement = Column(Float)  # Average height/thickness
    measurement_std_dev = Column(Float)  # Measurement variation
    overall_quality_score = Column(Float)  # 0-100 quality rating
    
    # Defect summary
    defect_count = Column(Integer, default=0)
    critical_defect_count = Column(Integer, default=0)
    warning_defect_count = Column(Integer, default=0)
    
    # Timing
    created_at = Column(DateTime, default=datetime.utcnow)
    started_at = Column(DateTime, nullable=True)
    completed_at = Column(DateTime, nullable=True)
    
    # Relationships
    student = relationship("Student", foreign_keys=[student_id])
    configuration = relationship("ScanConfiguration", back_populates="scans")
    scan_points = relationship("ScanPoint", back_populates="scan", cascade="all, delete-orphan")
    measurements = relationship("Measurement", back_populates="scan", cascade="all, delete-orphan")
    defects = relationship("Defect", back_populates="scan", cascade="all, delete-orphan")
    
    def to_dict(self, include_points: bool = False) -> dict:
        """Convert to dictionary"""
        data = {
            'id': self.id,
            'scan_id': self.scan_id,
            'student_id': self.student_id,
            'status': self.status.value,
            'workpiece_type': self.workpiece_type,
            'workpiece_material': self.workpiece_material,
            'total_points': self.total_points,
            'completed_points': self.completed_points,
            'overall_quality_score': self.overall_quality_score,
            'defect_count': self.defect_count,
            'critical_defect_count': self.critical_defect_count,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'completed_at': self.completed_at.isoformat() if self.completed_at else None,
        }
        
        if include_points:
            data['scan_points'] = [p.to_dict() for p in self.scan_points]
            data['measurements'] = [m.to_dict() for m in self.measurements]
            data['defects'] = [d.to_dict() for d in self.defects]
        
        return data


class ScanPoint(Base):
    """Individual scan position and capture"""
    __tablename__ = 'scan_points'
    
    id = Column(Integer, primary_key=True)
    scan_id = Column(Integer, ForeignKey('scans.id'), nullable=False)
    
    # Position
    position_x = Column(Float)
    position_y = Column(Float)
    position_z = Column(Float)
    
    # Grid coordinates
    grid_x = Column(Integer)  # Grid cell X index
    grid_y = Column(Integer)  # Grid cell Y index
    
    # Capture result
    success = Column(Boolean, default=False)
    error_message = Column(String(200))
    
    # Frame information
    rgb_image_path = Column(String(255))  # Path to saved RGB image
    depth_image_path = Column(String(255))  # Path to saved depth map
    
    # Point cloud information
    point_count = Column(Integer, default=0)  # Points in cloud for this position
    point_cloud_path = Column(String(255))  # Path to saved point cloud (PCD or NPY)
    
    # Processing time
    capture_time_ms = Column(Float)
    processing_time_ms = Column(Float)
    
    # Timestamp
    captured_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    scan = relationship("Scan", back_populates="scan_points")
    measurement = relationship("Measurement", uselist=False, back_populates="scan_point")
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'id': self.id,
            'position': {
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z,
            },
            'grid_position': {
                'x': self.grid_x,
                'y': self.grid_y,
            },
            'success': self.success,
            'point_count': self.point_count,
            'capture_time_ms': self.capture_time_ms,
            'processing_time_ms': self.processing_time_ms,
        }


class Measurement(Base):
    """Weld measurement at a scan point"""
    __tablename__ = 'measurements'
    
    id = Column(Integer, primary_key=True)
    scan_id = Column(Integer, ForeignKey('scans.id'), nullable=False)
    scan_point_id = Column(Integer, ForeignKey('scan_points.id'), nullable=True)
    
    # Measurement values
    height_mm = Column(Float)  # Weld height/thickness
    width_mm = Column(Float)  # Weld width
    area_mm2 = Column(Float)  # Weld cross-section area
    
    # Statistics
    height_min = Column(Float)
    height_max = Column(Float)
    height_mean = Column(Float)
    height_std_dev = Column(Float)
    
    # Quality metrics
    uniformity_score = Column(Float)  # 0-100 (higher = more uniform)
    symmetry_score = Column(Float)  # 0-100 (higher = more symmetric)
    penetration_estimate = Column(Float)  # Estimated penetration percentage
    
    # Material coverage
    material_percentage = Column(Float)  # % of area with material (vs void)
    
    # Timestamp
    measured_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    scan = relationship("Scan", back_populates="measurements")
    scan_point = relationship("ScanPoint", back_populates="measurement")
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'id': self.id,
            'measurements': {
                'height_mm': self.height_mm,
                'width_mm': self.width_mm,
                'area_mm2': self.area_mm2,
            },
            'statistics': {
                'height_min': self.height_min,
                'height_max': self.height_max,
                'height_mean': self.height_mean,
                'height_std_dev': self.height_std_dev,
            },
            'quality': {
                'uniformity_score': self.uniformity_score,
                'symmetry_score': self.symmetry_score,
                'penetration_estimate': self.penetration_estimate,
                'material_percentage': self.material_percentage,
            },
        }


class Defect(Base):
    """Defect detected during scanning"""
    __tablename__ = 'defects'
    
    id = Column(Integer, primary_key=True)
    scan_id = Column(Integer, ForeignKey('scans.id'), nullable=False)
    
    # Defect information
    defect_type = Column(SQLEnum(DefectType), nullable=False)
    severity = Column(Float)  # 0-100 severity score
    confidence = Column(Float)  # 0-100 confidence in detection
    
    # Location
    position_x = Column(Float)
    position_y = Column(Float)
    position_z = Column(Float)
    
    # Spatial extent
    center_x = Column(Float)  # Center in image coordinates
    center_y = Column(Float)
    width_mm = Column(Float)
    height_mm = Column(Float)
    area_mm2 = Column(Float)
    
    # Description
    description = Column(Text)
    
    # Classification
    is_critical = Column(Boolean, default=False)  # Critical defect
    is_actionable = Column(Boolean, default=False)  # Requires action
    
    # Detection metadata
    detected_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    scan = relationship("Scan", back_populates="defects")
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            'id': self.id,
            'defect_type': self.defect_type.value,
            'severity': self.severity,
            'confidence': self.confidence,
            'position': {
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z,
            },
            'dimensions': {
                'width_mm': self.width_mm,
                'height_mm': self.height_mm,
                'area_mm2': self.area_mm2,
            },
            'is_critical': self.is_critical,
            'is_actionable': self.is_actionable,
            'description': self.description,
        }
