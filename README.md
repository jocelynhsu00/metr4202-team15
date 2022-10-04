# METR4202 REPO

Branches:

logic: logic + state machine, deciding which blocks to pick up, obstacle avoidance, predicting block position, deciding which scripts to call

controller: publishing to dynamixels: given in wk7 UQ-METR4202 code

camera: reading where the blocks are, provide position of block to logic to decide an end effector position

inverse_kinematics: inv kin, compare to for kin, provide 2 solutions for joint angles given an end effector position

gripper: open close gripper (given in prac code, call when robot has moved to supplied end effector position)



## REMINDERS: 
Do not commit to main for now, please commit to your branch

Keep commits when merging after pull request

### Branch names: controller, inverse_kinematics, forward_kinematics, camera, gripper

### Git commands:

git checkout "insert branch name"

git clone insert link: to clone a repo 

git pull: to pull new changes

git status: to see modifications

git add insert path/file_name: to add a file to be committed

git commit -m "insert commit msg": to stage a commit

git push: to push the commit 

