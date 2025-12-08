"""
Database models for training and models
"""

from sqlalchemy import Column, Integer, String, Float, DateTime, Boolean, Text
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime

Base = declarative_base()


class TrainingJob(Base):
    """Training job record"""
    __tablename__ = 'training_jobs'
    
    id = Column(Integer, primary_key=True)
    job_id = Column(String(255), unique=True)
    model_type = Column(String(50))  # mobilenet, resnet50
    status = Column(String(50))  # running, completed, failed, cancelled
    epoch = Column(Integer, default=0)
    total_epochs = Column(Integer)
    loss = Column(Float)
    accuracy = Column(Float)
    progress = Column(Integer, default=0)
    started_at = Column(DateTime, default=datetime.now)
    completed_at = Column(DateTime)
    error_message = Column(Text)


class Model(Base):
    """Trained model record"""
    __tablename__ = 'models'
    
    id = Column(Integer, primary_key=True)
    name = Column(String(255), unique=True)
    model_type = Column(String(50))  # mobilenet, resnet50, ensemble
    version = Column(Integer)
    accuracy = Column(Float)
    loss = Column(Float)
    file_path = Column(String(500))
    onnx_path = Column(String(500))
    is_active = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.now)
    training_job_id = Column(String(255))
