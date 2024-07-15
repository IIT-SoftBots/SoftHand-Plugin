import mujoco
import mujoco.viewer
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('/home/esguerri/SoftHand/src/SoftHand-Plugin/softhands_description/urdf/xml_for_mujoco/export/right_softhand_v1_2_research_edited.xml')
data = mujoco.MjData(model)

# Set up the viewer
viewer = mujoco.viewer.launch(model, data)

while True:
    mujoco.mj_step(model, data)
    viewer.sync()  # Sync the viewer with the simulation
