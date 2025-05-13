"""
Add variables to the MATLAB workspace using the engine's workspace dictionary interface.
Call MATLAB functions that use these variables.
Retrieve results from MATLAB if needed.
"""
import matlab.engine
import numpy as np

# Start the MATLAB engine
eng = matlab.engine.start_matlab()

# MATLAB Engine runs functions, not scripts
data = eng.matlabEngineTest()
#print(data)

"""
# Create sample data in Python
python_list = [1, 2, 3, 4, 5]
python_array = np.array([[1, 2], [3, 4]])
python_dict = {'a': 1, 'b': 2}

# Send data to MATLAB and call functions
mean_value = eng.mean(matlab.double(python_list))
sum_array = eng.sum(matlab.double(python_array.tolist()))
struct = eng.struct(python_dict)

# Print results
print(f"Mean of list: {mean_value}")
print(f"Sum of array: {sum_array}")
print(f"MATLAB struct: {struct}")
"""
# Close the MATLAB engine
eng.quit()


"""
Using pyrun in MATLAB:
This approach is suitable for running Python code snippets directly from MATLAB.
"""