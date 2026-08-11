import random
from pathlib import Path

import pandas as pd
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms
from torchvision.transforms import functional as transform_functional


class DrivingDataset(Dataset):
    def __init__(
        self,
        data,
        dataset_directory,
        augment=False,
        balance_straight=False
    ):
        self.dataset_directory = Path(dataset_directory)
        self.augment = augment

        # Each dataset receives its own copy.
        self.data = data.copy().reset_index(drop=True)

        required_columns = {
            "image_path",
            "steering"
        }

        missing_columns = (
            required_columns - set(self.data.columns)
        )

        if missing_columns:
            raise ValueError(
                f"Missing CSV columns: {missing_columns}"
            )

        if balance_straight:
            self._reduce_straight_samples()

        # Applied to both training and validation.
        # Must also match inference preprocessing.
        self.base_transform = transforms.Compose([
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
            transforms.Resize((180, 320)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.5, 0.5, 0.5],
                std=[0.5, 0.5, 0.5]
            )
        ])

        # Training-only augmentation.
        self.color_jitter = transforms.ColorJitter(
            brightness=0.15,
            contrast=0.15,
            saturation=0.10
        )

        print(
            f"DrivingDataset created | "
            f"samples={len(self.data)} | "
            f"augment={self.augment} | "
            f"balance_straight={balance_straight}"
        )

    def _reduce_straight_samples(self):
        """
        Keep all turning samples but only part of samples whose
        steering is very close to zero.
        """
        straight_threshold = 0.02
        straight_keep_fraction = 0.40

        straight_mask = (
            self.data["steering"].abs()
            < straight_threshold
        )

        straight_samples = self.data[straight_mask]
        turning_samples = self.data[~straight_mask]

        if len(straight_samples) > 0:
            straight_samples = straight_samples.sample(
                frac=straight_keep_fraction,
                random_state=42
            )

        self.data = pd.concat(
            [
                straight_samples,
                turning_samples
            ],
            ignore_index=True
        )

        # Shuffle only after filtering.
        self.data = self.data.sample(
            frac=1.0,
            random_state=42
        ).reset_index(drop=True)

        print(
            "Training balancing:"
            f" straight kept={len(straight_samples)},"
            f" turning kept={len(turning_samples)},"
            f" total={len(self.data)}"
        )

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        row = self.data.iloc[index]

        image_path = (
            self.dataset_directory
            / Path(row["image_path"])
        )

        if not image_path.exists():
            raise FileNotFoundError(
                f"Image does not exist: {image_path}"
            )

        image = Image.open(image_path).convert("RGB")
        steering = float(row["steering"])

        if self.augment:
            image = self.color_jitter(image)

            # Horizontal flip:
            # a right turn becomes a left turn and vice versa.
            if random.random() < 0.5:
                image = transform_functional.hflip(image)
                steering = -steering

        image_tensor = self.base_transform(image)

        steering_tensor = torch.tensor(
            steering,
            dtype=torch.float32
        )

        return image_tensor, steering_tensor