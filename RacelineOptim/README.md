### Step 1: Download pgm from Raspberry Pi onto computer

### Step 2: Clean up noise (and shh, maybe just maybe round the corners of the track just a little bit, shhh)

# DO NOT FORGET TO CHANGE PATHS AND TO USE CORRECT FILES FOR EVERYTHING. 

# DOUBLE CHECK, THEN DOUBLE DOUBLE CHECK. THIS COST US THE RACE IN 2024. 


### Step 3: Run the map_converter notebook to extract centerline+width from the map. 

### Step 4: Change the paths in main_globaltraj_tt02.py to use the newly generated files. 

### Step 5: Run it. 

### Step 6: Check the files with the visualize_raceline ipynb

### Step 7: Transfer the traj to the car, and DO NOT FORGET TO CHANGE THE PATH IN THE CONTROLLER FILE TO USE THE NEW TRAJECTORY

# I CANNOT STATE THIS ENOUGH. TRIPLE CHECK THAT YOU ARE USING THE CORRECT TRAJECTORY WITH THE CORRESPONDING MAP.

# YOU WILL RUN INTO THE FIRST WALL BEFORE YOU EVEN KNOW WHAT'S HAPPENING OTHERWISE. 

# GODSPEED.
