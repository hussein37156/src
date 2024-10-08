import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('/home/hussein/catkin_ws/src/Project 1/data_sets/train.csv')


image_data = df.iloc[1002, 1:].values.reshape(28, 28)  # Skip the first column



# Display the image
plt.imshow(image_data, cmap='gray')
plt.axis('on')  # Turn off axis numbers and ticks
plt.show()
