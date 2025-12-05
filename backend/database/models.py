from sqlalchemy import create_engine, Column, Integer, String, Float, ForeignKey, DateTime, Boolean
from sqlalchemy.orm import declarative_base, relationship, sessionmaker
from datetime import datetime
import json

Base = declarative_base()

class Student(Base):
    __tablename__ = 'students'
    
    id = Column(Integer, primary_key=True)
    name = Column(String, nullable=False)
    student_id = Column(String, unique=True, nullable=False)
    class_name = Column(String, nullable=False)
    level = Column(String, default="Novice")
    
    scans = relationship("Scan", back_populates="student")

class Scan(Base):
    __tablename__ = 'scans'
    
    id = Column(Integer, primary_key=True)
    student_id = Column(Integer, ForeignKey('students.id'))
    timestamp = Column(DateTime, default=datetime.now)
    
    # Metrics
    total_score = Column(Integer)
    width_val = Column(Float)
    height_val = Column(Float)
    uniformity_score = Column(Float)
    porosity_count = Column(Integer)
    spatter_count = Column(Integer)
    undercut_detected = Column(Boolean)
    
    defects_json = Column(String) # Store as JSON string
    image_path = Column(String)
    status = Column(String) # Pass/Fail
    
    student = relationship("Student", back_populates="scans")

def init_db(db_path='sqlite:///weld_data.db'):
    engine = create_engine(db_path)
    Base.metadata.create_all(engine)
    return sessionmaker(bind=engine)