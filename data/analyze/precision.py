import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import seaborn as sns

# libraries
import numpy as np
import matplotlib.pyplot as plt
 
# width of the bars
barWidth = 0.3
# Choose the height of the blue bars
bars2 = [0.3, 0.80, 0.77]
# Choose the height of the cyan bars
bars1 = [ 0.4,0.82, 0.93]
# Choose the height of the error bars (bars1)
yer1 = [ 0.13, 0.04,0.02]
# Choose the height of the error bars (bars2)
yer2 = [ 0.12,0.02, 0.01]
plt.rcParams["font.size"] = "35"
# The x position of bars
r1 = np.arange(len(bars1))
r2 = [x + barWidth for x in r1]
 
# Create blue bars
plt.bar(r1, bars1, width = barWidth, color = 'blue', edgecolor = 'black', yerr=yer1, capsize=7, label='RoI')
# Create cyan bars
plt.bar(r2, bars2, width = barWidth, color = 'cyan', edgecolor = 'black', yerr=yer2, capsize=7, label='Not RoI')
# general layout
plt.xticks([r + barWidth for r in range(len(bars1))], ['PCL', 'Proposed', 'Zermas et al. 2017'])
plt.ylabel('F1 score')
plt.legend()
 
# Show graphic
plt.show()
