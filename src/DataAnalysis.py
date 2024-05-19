import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data from Excel file
df = pd.read_excel('src\MADS_runtime_comparison_Parallelisation.xlsx', engine='openpyxl', usecols=range(10))

# Create a figure and axis object
fig, ax = plt.subplots()

# Iterate over each column in the DataFrame
for column in df.columns:
    # Create a box plot for each column
    ax.boxplot(df[column], positions=[df.columns.get_loc(column) + 1], widths=0.6, showfliers = False)

# Set x-axis labels
plt.xticks(range(1, len(df.columns) + 1), df.columns)

# Set plot title and labels
#plt.title('Effect of Area Discretization Mesh Coarsening on Computation Time')
plt.xlabel('Number of Threads')
plt.ylabel('Optimization Runtime (s)')

# Show plot
plt.show()



# Calculate the median for each column
medians = df.median()

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(np.log(medians.index), np.log(medians.values), marker='o', linestyle='-', color='b')
plt.xlabel('$Log(d_m)$',fontsize=15)
plt.ylabel('$Log(T)$',fontsize=15)
#plt.title('Polynomial relationship between computation time and mesh size',fontsize=15)
plt.grid(True)
#plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()

# Fit a linear function to the log-transformed median values
x = np.log(medians.index).astype(float)  # Ensure x values are floats
y = np.log(medians.values)
coefficients = np.polyfit(x, y, 1)  # Fit a first-degree polynomial (linear fit)
slope, intercept = coefficients
print(slope)
print(intercept)


