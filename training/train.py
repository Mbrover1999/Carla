from pathlib import Path
import random

import numpy as np
import pandas as pd
import torch
from torch import nn
from torch.utils.data import DataLoader

from driving_dataset import DrivingDataset
from model import SteeringModel


PROJECT_DIRECTORY = (
    Path(__file__).resolve().parent.parent
)

DATASET_DIRECTORY = (
    PROJECT_DIRECTORY / "dataset"
)

CSV_PATH = (
    DATASET_DIRECTORY / "driving_log.csv"
)

MODEL_DIRECTORY = (
    PROJECT_DIRECTORY / "trained_models"
)

# Save as a new model instead of overwriting v1.
MODEL_PATH = (
    MODEL_DIRECTORY
    / "steering_model_v2.pth"
)


BATCH_SIZE = 64
NUMBER_OF_EPOCHS = 15
LEARNING_RATE = 0.0005

VALIDATION_RATIO = 0.20
RANDOM_SEED = 42

# Leave a small separation between training and validation
# so nearly adjacent frames are less likely to leak across.
VALIDATION_GAP = 20


def set_random_seed():
    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)
    torch.manual_seed(RANDOM_SEED)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(
            RANDOM_SEED
        )


def choose_device():
    if torch.cuda.is_available():
        return torch.device("cuda")

    if (
        hasattr(torch.backends, "mps")
        and torch.backends.mps.is_available()
    ):
        return torch.device("mps")

    return torch.device("cpu")


def load_and_split_data():
    data = pd.read_csv(CSV_PATH)

    if len(data) < 100:
        raise RuntimeError(
            "The dataset contains too few samples"
        )

    validation_size = int(
        len(data) * VALIDATION_RATIO
    )

    validation_start = (
        len(data) - validation_size
    )

    training_end = max(
        0,
        validation_start - VALIDATION_GAP
    )

    training_data = data.iloc[
        :training_end
    ].copy()

    validation_data = data.iloc[
        validation_start:
    ].copy()

    if len(training_data) == 0:
        raise RuntimeError(
            "Training split is empty"
        )

    if len(validation_data) == 0:
        raise RuntimeError(
            "Validation split is empty"
        )

    return training_data, validation_data


def evaluate(
    model,
    data_loader,
    loss_function,
    device
):
    model.eval()

    total_loss = 0.0
    total_absolute_error = 0.0
    total_samples = 0

    with torch.inference_mode():
        for images, steering in data_loader:
            images = images.to(
                device,
                non_blocking=True
            )

            steering = steering.to(
                device,
                non_blocking=True
            )

            predictions = model(images)

            loss = loss_function(
                predictions,
                steering
            )

            batch_size = images.size(0)

            total_loss += (
                loss.item() * batch_size
            )

            total_absolute_error += (
                torch.abs(
                    predictions - steering
                ).sum().item()
            )

            total_samples += batch_size

    if total_samples == 0:
        return 0.0, 0.0

    average_loss = (
        total_loss / total_samples
    )

    mean_absolute_error = (
        total_absolute_error / total_samples
    )

    return (
        average_loss,
        mean_absolute_error
    )


def main():
    set_random_seed()

    training_data, validation_data = (
        load_and_split_data()
    )

    training_dataset = DrivingDataset(
        data=training_data,
        dataset_directory=DATASET_DIRECTORY,
        augment=True,
        balance_straight=True
    )

    validation_dataset = DrivingDataset(
        data=validation_data,
        dataset_directory=DATASET_DIRECTORY,
        augment=False,
        balance_straight=False
    )

    training_loader = DataLoader(
        training_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=0,
        pin_memory=torch.cuda.is_available()
    )

    validation_loader = DataLoader(
        validation_dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=0,
        pin_memory=torch.cuda.is_available()
    )

    device = choose_device()

    print()
    print(f"Using device: {device}")

    if device.type == "cuda":
        print(
            "GPU:",
            torch.cuda.get_device_name(0)
        )

    print(
        f"Original training rows: "
        f"{len(training_data)}"
    )

    print(
        f"Balanced training samples: "
        f"{len(training_dataset)}"
    )

    print(
        f"Validation samples: "
        f"{len(validation_dataset)}"
    )

    model = SteeringModel().to(device)

    # More robust than MSE for occasional unusual labels.
    loss_function = nn.SmoothL1Loss(
        beta=0.05
    )

    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=LEARNING_RATE,
        weight_decay=1e-4
    )

    scheduler = (
        torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer,
            mode="min",
            factor=0.5,
            patience=2
        )
    )

    MODEL_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True
    )

    best_validation_loss = float("inf")

    for epoch in range(NUMBER_OF_EPOCHS):
        model.train()

        total_training_loss = 0.0
        total_training_samples = 0

        for images, steering in training_loader:
            images = images.to(
                device,
                non_blocking=True
            )

            steering = steering.to(
                device,
                non_blocking=True
            )

            optimizer.zero_grad(
                set_to_none=True
            )

            predictions = model(images)

            loss = loss_function(
                predictions,
                steering
            )

            loss.backward()

            # Prevent unusually large gradient updates.
            torch.nn.utils.clip_grad_norm_(
                model.parameters(),
                max_norm=1.0
            )

            optimizer.step()

            batch_size = images.size(0)

            total_training_loss += (
                loss.item() * batch_size
            )

            total_training_samples += (
                batch_size
            )

        training_loss = (
            total_training_loss
            / total_training_samples
        )

        (
            validation_loss,
            validation_mae
        ) = evaluate(
            model=model,
            data_loader=validation_loader,
            loss_function=loss_function,
            device=device
        )

        scheduler.step(validation_loss)

        current_learning_rate = (
            optimizer.param_groups[0]["lr"]
        )

        print(
            f"Epoch "
            f"{epoch + 1:02d}/"
            f"{NUMBER_OF_EPOCHS} | "
            f"train_loss={training_loss:.6f} | "
            f"validation_loss="
            f"{validation_loss:.6f} | "
            f"validation_MAE="
            f"{validation_mae:.6f} | "
            f"lr={current_learning_rate:.7f}"
        )

        if validation_loss < best_validation_loss:
            best_validation_loss = (
                validation_loss
            )

            torch.save(
                model.state_dict(),
                MODEL_PATH
            )

            print(
                f"Saved improved model: "
                f"{MODEL_PATH}"
            )

    print()
    print("Training completed")
    print(
        f"Best validation loss: "
        f"{best_validation_loss:.6f}"
    )
    print(f"Model saved at: {MODEL_PATH}")


if __name__ == "__main__":
    main()