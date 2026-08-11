import torch
from torch import nn


class SteeringModel(nn.Module):
    def __init__(self):
        super().__init__()

        self.features = nn.Sequential(
            nn.Conv2d(
                in_channels=3,
                out_channels=24,
                kernel_size=5,
                stride=2
            ),
            nn.ReLU(),

            nn.Conv2d(
                in_channels=24,
                out_channels=36,
                kernel_size=5,
                stride=2
            ),
            nn.ReLU(),

            nn.Conv2d(
                in_channels=36,
                out_channels=48,
                kernel_size=5,
                stride=2
            ),
            nn.ReLU(),

            nn.Conv2d(
                in_channels=48,
                out_channels=64,
                kernel_size=3,
                stride=2
            ),
            nn.ReLU(),

            nn.AdaptiveAvgPool2d((4, 8))
        )

        self.regressor = nn.Sequential(
            nn.Flatten(),

            nn.Linear(64 * 4 * 8, 256),
            nn.ReLU(),
            nn.Dropout(0.25),

            nn.Linear(256, 64),
            nn.ReLU(),

            nn.Linear(64, 1),
            nn.Tanh()
        )

    def forward(self, images):
        features = self.features(images)
        steering = self.regressor(features)

        return steering.squeeze(1)