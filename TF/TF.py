import tensorflow as tf
from tensorflow.keras import layers, models # type: ignore

# Define a simple model
model = models.Sequential([
    layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(64, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Flatten(),
    layers.Dense(64, activation='relu'),
    layers.Dense(10, activation='softmax')
])

# Compile the model
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Print model summary
model.summary()

# Create some dummy data
import numpy as np

x_train = np.random.random((1000, 28, 28, 1))
y_train = np.random.randint(10, size=(1000,))

# Train the model
model.fit(x_train, y_train, epochs=5, batch_size=32)

# Evaluate the model
loss, acc = model.evaluate(x_train, y_train)
print(f"Loss: {loss}, Accuracy: {acc}")
