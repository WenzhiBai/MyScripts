import os
import json

WorkPath = "D:/TestAndAnalysis/Data"

Data = os.listdir("D:/TestAndAnalysis/Data")

for Scene in Data:
    #os.rename(WorkPath+"/"+Scene+r'/LocalizingData',WorkPath+"/"+Scene+r'/RelocalizationData')
    RelocalizationData = os.listdir(WorkPath+"/"+Scene+"/RelocalizationData")
    for Position in RelocalizationData:
        PositionData = os.listdir(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/")
        #os.rename(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"Pose.json",
        #          WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"RefPose.json")
        for file in PositionData:
            if file == 'RefPose.json':
                with open(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"RefPose.json",'r') as PoseRead:
                    PoseData = json.load(PoseRead)
                    PoseData['x'] = PoseData['poseX']
                    PoseData['y'] = PoseData['poseY']
                    PoseData['phi'] = PoseData['posePhi']
                    del PoseData['poseX']
                    del PoseData['poseY']
                    del PoseData['posePhi']
                    print(PoseData)

                with open(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"RefPose.json",'w') as PoseWrite:
                    json.dump(PoseData,PoseWrite)

            if file == 'PredictPose.json':
                with open(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"PredictPose.json",'r') as PoseRead:
                    PoseData = json.load(PoseRead)
                    PoseData['x'] = PoseData['predictPoseX']
                    PoseData['y'] = PoseData['predictPoseY']
                    PoseData['phi'] = PoseData['predictPosePhi']
                    del PoseData['predictPoseX']
                    del PoseData['predictPoseY']
                    del PoseData['predictPosePhi']
                    print(PoseData)

                with open(WorkPath+"/"+Scene+"/RelocalizationData"+"/"+Position+"/"+"PredictPose.json",'w') as PoseWrite:
                    json.dump(PoseData,PoseWrite)
