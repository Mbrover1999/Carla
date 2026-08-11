from pathlib import Path
import random

import cv2
import pandas as pd

from inference.steering_predictor import SteeringPredictor


PROJECT_ROOT = Path(__file__).resolve().parent.parent

MODEL_PATH = (
    PROJECT_ROOT
    / "trained_models"
    / "steering_model_v2.pth"
)

DATASET_DIRECTORY = PROJECT_ROOT / "dataset"
CSV_PATH = DATASET_DIRECTORY / "driving_log.csv"

NUMBER_OF_SAMPLES = 30
RANDOM_SEED = 42


def choose_sample_indices(data_size, number_of_samples):
    random.seed(RANDOM_SEED)

    number_of_samples = min(
        number_of_samples,
        data_size
    )

    return random.sample(
        range(data_size),
        number_of_samples
    )


def steering_direction(value):
    if value < -0.05:
        return "LEFT"

    if value > 0.05:
        return "RIGHT"

    return "STRAIGHT"


def draw_prediction(
    image,
    real_steering,
    predicted_steering,
    sample_number,
    total_samples
):
    display_image = image.copy()

    absolute_error = abs(
        real_steering - predicted_steering
    )

    lines = [
        f"Sample: {sample_number}/{total_samples}",
        f"Real:      {real_steering:+.4f} "
        f"({steering_direction(real_steering)})",
        f"Predicted: {predicted_steering:+.4f} "
        f"({steering_direction(predicted_steering)})",
        f"Abs error: {absolute_error:.4f}",
        "SPACE/N: next | Q/ESC: quit"
    ]

    overlay = display_image.copy()

    cv2.rectangle(
        overlay,
        (10, 10),
        (570, 175),
        (0, 0, 0),
        thickness=-1
    )

    cv2.addWeighted(
        overlay,
        0.65,
        display_image,
        0.35,
        0,
        display_image
    )

    y_position = 40

    for line in lines:
        cv2.putText(
            display_image,
            line,
            (25, y_position),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        y_position += 30

    return display_image


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(
            f"CSV file was not found: {CSV_PATH}"
        )

    data = pd.read_csv(CSV_PATH)

    required_columns = {
        "image_path",
        "steering"
    }

    missing_columns = required_columns - set(data.columns)

    if missing_columns:
        raise ValueError(
            f"Missing CSV columns: {missing_columns}"
        )

    predictor = SteeringPredictor(
        model_path=MODEL_PATH
    )

    sample_indices = choose_sample_indices(
        data_size=len(data),
        number_of_samples=NUMBER_OF_SAMPLES
    )

    errors = []

    for sample_number, row_index in enumerate(
        sample_indices,
        start=1
    ):
        row = data.iloc[row_index]

        image_path = (
            DATASET_DIRECTORY
            / Path(row["image_path"])
        )

        if not image_path.exists():
            print(
                f"Skipping missing image: {image_path}"
            )
            continue

        real_steering = float(row["steering"])

        predicted_steering = (
            predictor.predict_from_path(image_path)
        )

        absolute_error = abs(
            real_steering - predicted_steering
        )

        errors.append(absolute_error)

        image = cv2.imread(str(image_path))

        if image is None:
            print(
                f"OpenCV could not open: {image_path}"
            )
            continue

        display_image = draw_prediction(
            image=image,
            real_steering=real_steering,
            predicted_steering=predicted_steering,
            sample_number=sample_number,
            total_samples=len(sample_indices)
        )

        cv2.imshow(
            "Steering Model Evaluation",
            display_image
        )

        print(
            f"Sample {sample_number:02d} | "
            f"real={real_steering:+.4f} | "
            f"predicted={predicted_steering:+.4f} | "
            f"error={absolute_error:.4f}"
        )

        while True:
            key = cv2.waitKey(0) & 0xFF

            if key in (
                ord(" "),
                ord("n"),
                ord("N")
            ):
                break

            if key in (
                ord("q"),
                ord("Q"),
                27
            ):
                cv2.destroyAllWindows()
                print_summary(errors)
                return

    cv2.destroyAllWindows()
    print_summary(errors)


def print_summary(errors):
    if not errors:
        print("No samples were evaluated")
        return

    mean_absolute_error = sum(errors) / len(errors)
    maximum_error = max(errors)

    print()
    print("Evaluation summary")
    print("------------------")
    print(f"Samples evaluated: {len(errors)}")
    print(f"Mean absolute error: {mean_absolute_error:.6f}")
    print(f"Maximum absolute error: {maximum_error:.6f}")


if __name__ == "__main__":
    main()