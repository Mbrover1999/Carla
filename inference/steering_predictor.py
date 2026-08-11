from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from torchvision import transforms

from config import (
    MODEL_IMAGE_HEIGHT,
    MODEL_IMAGE_WIDTH
)
from training.model import SteeringModel


class SteeringPredictor:
    def __init__(self, model_path):
        self.model_path = Path(model_path)

        if not self.model_path.exists():
            raise FileNotFoundError(
                f"Model file was not found: {self.model_path}"
            )

        self.device = self._choose_device()

        self.transform = transforms.Compose([
            transforms.Lambda(
                lambda image: image.crop(
                    (
                        0,
                        int(image.height * 0.35),
                        image.width,
                        image.height
                    )
                )
            ),
            transforms.Resize((
                MODEL_IMAGE_HEIGHT,
                MODEL_IMAGE_WIDTH
            )),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.5, 0.5, 0.5],
                std=[0.5, 0.5, 0.5]
            )
        ])

        self.model = SteeringModel().to(self.device)

        state_dict = torch.load(
            self.model_path,
            map_location=self.device,
            weights_only=True
        )

        self.model.load_state_dict(state_dict)
        self.model.eval()

        print(f"Steering model loaded on: {self.device}")

    def predict_from_path(self, image_path):
        image_path = Path(image_path)

        if not image_path.exists():
            raise FileNotFoundError(
                f"Image was not found: {image_path}"
            )

        image = Image.open(image_path).convert("RGB")

        return self.predict_from_pil(image)

    def predict_from_numpy(self, frame):
        if frame is None:
            raise ValueError("The input frame cannot be None")

        if not isinstance(frame, np.ndarray):
            raise TypeError(
                "The input frame must be a NumPy array"
            )

        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(
                "Expected an image with shape (height, width, 3)"
            )

        # sensors.py produces BGR for OpenCV.
        # PIL and the trained model expect RGB.
        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        image = Image.fromarray(rgb_frame)

        return self.predict_from_pil(image)

    def predict_from_pil(self, image):
        image_tensor = self.transform(image)
        image_tensor = image_tensor.unsqueeze(0)
        image_tensor = image_tensor.to(self.device)

        # Inference mode disables gradient tracking during prediction.
        with torch.inference_mode():
            prediction = self.model(image_tensor)

        return float(prediction.item())

    @staticmethod
    def _choose_device():
        if torch.cuda.is_available():
            return torch.device("cuda")

        return torch.device("cpu")