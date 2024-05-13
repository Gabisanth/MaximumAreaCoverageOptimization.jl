import pandas as pd
import matplotlib.pyplot as plt

# Read data from Excel file
df = pd.read_excel('src\MADS_runtime_comparison_GranularVariables.xlsx', engine='openpyxl')

# Create a figure and axis object
fig, ax = plt.subplots()

# Iterate over each column in the DataFrame
for column in df.columns:
    # Create a box plot for each column
    ax.boxplot(df[column], positions=[df.columns.get_loc(column) + 1], widths=0.6, showfliers = False)

# Set x-axis labels
plt.xticks(range(1, len(df.columns) + 1), df.columns)

# Set plot title and labels
plt.title('Effect of Granular Variables on Computation Time')
plt.xlabel('Granular Variable Setting')
plt.ylabel('Optimization Runtime (s)')

# Show plot
plt.show()


