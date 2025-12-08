"""
Training Pipeline for Desktop
ResNet50 + MobileNet training for improved accuracy
"""

import torch
import torch.nn as nn
import torchvision.models as models
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import numpy as np
import cv2
import logging
from datetime import datetime
from pathlib import Path
import json
from typing import Dict, List, Tuple

logger = logging.getLogger(__name__)


class WeldDefectDataset(Dataset):
    """Dataset for weld defect images"""
    
    def __init__(self, frames_dir: str, labels_file: str, transform=None):
        self.frames_dir = Path(frames_dir)
        self.transform = transform
        self.samples = []
        self.classes = {'good': 0, 'porosity': 1, 'undercut': 2}
        
        # Load labels
        with open(labels_file, 'r') as f:
            data = json.load(f)
            for item in data:
                self.samples.append((item['frame'], item['label']))
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        frame_path, label = self.samples[idx]
        
        # Load RGB and depth
        rgb = cv2.imread(str(self.frames_dir / frame_path))
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        
        # Load depth (simplified - would be actual depth data)
        depth_path = str(self.frames_dir / frame_path).replace('.jpg', '_depth.npy')
        depth = np.load(depth_path) if Path(depth_path).exists() else np.zeros((1080, 1920))
        
        # Preprocess
        rgb_resized = cv2.resize(rgb, (224, 224))
        depth_resized = cv2.resize(depth, (224, 224))
        
        # Normalize
        rgb_norm = rgb_resized / 255.0
        depth_min, depth_max = depth_resized.min(), depth_resized.max()
        if depth_max > depth_min:
            depth_norm = (depth_resized - depth_min) / (depth_max - depth_min)
        else:
            depth_norm = np.zeros_like(depth_resized)
        
        # Stack channels (6 channels: RGB + Depth x3)
        combined = np.stack([
            rgb_norm[:, :, 0],
            rgb_norm[:, :, 1],
            rgb_norm[:, :, 2],
            depth_norm,
            depth_norm,
            depth_norm
        ], axis=0).astype(np.float32)
        
        # Convert to tensor
        img_tensor = torch.from_numpy(combined)
        label_tensor = torch.tensor(self.classes[label], dtype=torch.long)
        
        return img_tensor, label_tensor


class TrainingPipeline:
    """Training pipeline for models"""
    
    def __init__(self, device='cuda' if torch.cuda.is_available() else 'cpu'):
        self.device = device
        self.models = {}
        self.training_history = {}
        logger.info(f"Training pipeline initialized on {device}")
    
    def create_mobilenet(self) -> nn.Module:
        """Create MobileNetV2 for edge deployment"""
        model = models.mobilenet_v2(pretrained=True)
        
        # Modify classifier for 3 classes
        model.classifier[1] = nn.Linear(1280, 3)
        
        return model.to(self.device)
    
    def create_resnet50(self) -> nn.Module:
        """Create ResNet50 for high accuracy"""
        model = models.resnet50(pretrained=True)
        
        # Modify final layer
        model.fc = nn.Linear(2048, 3)
        
        return model.to(self.device)
    
    def train_model(
        self,
        model_name: str,
        train_loader: DataLoader,
        val_loader: DataLoader,
        epochs: int = 100,
        learning_rate: float = 0.001
    ) -> Dict:
        """Train a model"""
        
        # Get or create model
        if model_name == 'mobilenet':
            model = self.create_mobilenet()
        elif model_name == 'resnet50':
            model = self.create_resnet50()
        else:
            raise ValueError(f"Unknown model: {model_name}")
        
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        criterion = nn.CrossEntropyLoss()
        
        history = {
            'epochs': [],
            'train_losses': [],
            'train_accs': [],
            'val_losses': [],
            'val_accs': []
        }
        
        best_acc = 0
        best_model_state = None
        
        logger.info(f"Starting {model_name} training for {epochs} epochs")
        
        for epoch in range(epochs):
            # Training phase
            model.train()
            train_loss = 0.0
            train_correct = 0
            train_total = 0
            
            for rgb_batch, labels in train_loader:
                rgb_batch = rgb_batch.to(self.device)
                labels = labels.to(self.device)
                
                # Forward
                outputs = model(rgb_batch)
                loss = criterion(outputs, labels)
                
                # Backward
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                # Stats
                train_loss += loss.item() * rgb_batch.size(0)
                _, predicted = torch.max(outputs, 1)
                train_correct += (predicted == labels).sum().item()
                train_total += labels.size(0)
            
            train_loss = train_loss / train_total
            train_acc = train_correct / train_total
            
            # Validation phase
            model.eval()
            val_loss = 0.0
            val_correct = 0
            val_total = 0
            
            with torch.no_grad():
                for rgb_batch, labels in val_loader:
                    rgb_batch = rgb_batch.to(self.device)
                    labels = labels.to(self.device)
                    
                    outputs = model(rgb_batch)
                    loss = criterion(outputs, labels)
                    
                    val_loss += loss.item() * rgb_batch.size(0)
                    _, predicted = torch.max(outputs, 1)
                    val_correct += (predicted == labels).sum().item()
                    val_total += labels.size(0)
            
            val_loss = val_loss / val_total
            val_acc = val_correct / val_total
            
            # Record history
            history['epochs'].append(epoch + 1)
            history['train_losses'].append(train_loss)
            history['train_accs'].append(train_acc)
            history['val_losses'].append(val_loss)
            history['val_accs'].append(val_acc)
            
            # Log progress
            if (epoch + 1) % 10 == 0:
                logger.info(
                    f"Epoch {epoch+1}/{epochs}: "
                    f"Loss={train_loss:.4f}, Acc={train_acc:.2%} | "
                    f"Val Loss={val_loss:.4f}, Val Acc={val_acc:.2%}"
                )
            
            # Save best model
            if val_acc > best_acc:
                best_acc = val_acc
                best_model_state = model.state_dict().copy()
        
        # Load best model
        if best_model_state:
            model.load_state_dict(best_model_state)
        
        # Save model
        model_path = f"backend/models/{model_name}_trained.pth"
        torch.save(model.state_dict(), model_path)
        logger.info(f"Model saved to {model_path}")
        
        self.training_history[model_name] = history
        return history
    
    def export_to_onnx(self, model_name: str, output_path: str = None):
        """Export model to ONNX format"""
        if model_name not in self.models:
            logger.error(f"Model {model_name} not found")
            return
        
        if output_path is None:
            output_path = f"backend/models/{model_name}.onnx"
        
        model = self.models[model_name]
        dummy_input = torch.randn(1, 6, 224, 224).to(self.device)
        
        torch.onnx.export(
            model, dummy_input, output_path,
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={'input': {0: 'batch_size'}}
        )
        
        logger.info(f"Model exported to {output_path}")
    
    def get_history(self, model_name: str) -> Dict:
        """Get training history"""
        return self.training_history.get(model_name, {})
