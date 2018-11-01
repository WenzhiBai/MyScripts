import os
import json

WorkPath = "D:/TestAndAnalysis/"
DataPath = WorkPath + "Data/"
AnalysisPath = WorkPath + "RelocalizationAnalysis/"
RelocalizationDataMidPath = "/RelocalizationData/"

DataFileName = "Data.csv"
ROCFileName = "ROC.png"
LearningCurveFileName = "LearningCurve.png"
RefPoseFileName = "RefPose.json"
PredictPoseFileName = "PredictPose.json"
EigenVectorFileName = "EigenVector.json"

MoveThreshold = 5
RotateThreshole = 5

EigenNames = []
EigenSpace = []
Lable = []

Data = os.listdir(DataPath)
for Scene in Data:
    RelocalizationData = os.listdir(DataPath + Scene + RelocalizationDataMidPath)
    for Position in RelocalizationData:
        PositionData = os.listdir(DataPath + Scene + RelocalizationDataMidPath + Position + "/")
        PoseData = []
        for JsonFile in PositionData:
            if JsonFile == RefPoseFileName:
                with open(DataPath + Scene + RelocalizationDataMidPath + Position + "/" + JsonFile,'r') as RefPoseRead:
                    PoseData.append(json.load(RefPoseRead))
                RefPoseRead.close()

            if JsonFile == PredictPoseFileName:
                with open(DataPath + Scene + RelocalizationDataMidPath + Position + "/" + JsonFile,'r') as PredictPoseRead:
                    PoseData.append(json.load(PredictPoseRead))
                PredictPoseRead.close()

            if JsonFile == EigenVectorFileName:
                with open(DataPath + Scene + RelocalizationDataMidPath + Position + "/" + JsonFile,'r') as EigenVectorRead:
                    EigenVectorData = json.load(EigenVectorRead)
                    if len(EigenNames) == 0:
                        EigenNames = list(EigenVectorData.keys())
                    EigenVector = []
                    for EigenItem in EigenNames:
                        EigenVector.append(EigenVectorData[EigenItem])
                    EigenSpace.append(EigenVector)
                EigenVectorRead.close()
                    
        if(abs(PoseData[0]['x']-PoseData[1]['x'])<MoveThreshold and
            abs(PoseData[0]['y']-PoseData[1]['y'])<MoveThreshold and
            abs(PoseData[0]['phi']-PoseData[1]['phi'])<RotateThreshole):
            Lable.append(1)
        else:
            Lable.append(0)

print(EigenSpace)

try:
    DataFile=open(AnalysisPath+DataFileName,'w')
    for EigenItem in EigenNames:
        DataFile.write(EigenItem)
        DataFile.write(",")
    DataFile.write("Lable")
    DataFile.write(",\n")

    cnt = 0
    for EigenVector in EigenSpace:
        for EigenItem in EigenVector:
            DataFile.write(str(EigenItem))
            DataFile.write(",")
        DataFile.write(str(Lable[cnt]))
        DataFile.write(",\n")
        cnt = cnt + 1
except Exception :
    print("Write DataFile Fail!!!")
finally:
    DataFile.close();

