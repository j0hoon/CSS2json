'''
Annotation이 완료된 데이터를 json 파일로 변환하는 코드입니다.

실행하기 위해 필요한 파일
- SF_PP.mat
- 원본 파일 (ex. CN7_030323_069.bag, RG3_030323_069.mf4)
- mat 파일 (ex. CN7_030323_069.mat)
- Annotation 파일 (ex. Annotation_CN7_030323_069.xlsx)
- Registration 파일 (ex. Registration_CN7_030323_069.xlsx)
- Lane 파일 (ex. Lane_CN7_030323_069.xlsx)

output 
CSS.json
'''

import json
import pandas as pd
import numpy as np
import scipy.io
import string
import scipy.io 
import glob
import os
import pyrosbag as rb
import json
from pathlib import Path
from CSS2json_lib import *
import natsort
from tqdm import tqdm
# SELECT MODE
import mat73
import h5py
from datetime import datetime 



class MakeJson():
    def __init__(self,matDir):
        os.system('cls')
        print("########## MAKE JSON  ##########\n########## CODE START ##########")

        self.matDir = matDir
        self.type = self.CheckType() # RG3 or CN7 or ERROR TYPE
        self.rosbagDir = matDir.replace('Rosbag2Mat','Rosbag')     # original
        self.sfDir = matDir + r'\Perception\SF'
        self.rgDir = matDir + r'\Registration'
        self.manDir = self.matDir + r'\Decision\Maneuver'
        self.date = str(matDir[-6:])

        self.jsonDir = os.getcwd() + r'\json'  # get current working directory
        
        self.adminDataType = 'EXP-' +self.type
        self.adminSampleTime = '0.05'
        self.adminVersion = {
            "CSS":"0.9",
            "LDT":"",
            "SF":""
        }
        self.adminProjectName = " "

        self.adminCMGT = 0
        self.adminAESGT = 8
        self.MakeJsonPath(self.jsonDir, "json")
        self.GeoreferenceType = "Point"
        self.TRACKNUM = 64
        self.FT_ID = 25
        self.FT_RELX = 29

    def AutoCuration(self):
        print("##########  "+str(self.type)+" AUTO  ##########")
        
        if self.type == 'CN7':
            rawDataDir = self.matDir.replace('Rosbag2Mat','Rosbag')
        elif self.type == 'RG3':
            rawDataDir = self.matDir.replace('mat','mf4')
        else :
            raise ("CAR TYPE ERROR!!")

        sfList = natsort.natsorted(glob.glob(self.sfDir + '\\*.mat'))
        matList =  natsort.natsorted(glob.glob(self.matDir + '\\*.mat'))
        if self.type == 'RG3':
            rawDataList = natsort.natsorted(glob.glob(rawDataDir + '\\*.mf4'))
        elif self.type == 'CN7':
            rawDataList = natsort.natsorted(glob.glob(rawDataDir + '\\*.bag'))
        
        man_list = os.listdir(self.manDir)
        
        
        for _man_path in tqdm(man_list):
            
            fnum = (_man_path.split("\\")[-1]).split("_")[3][:3]
            num = int(fnum) -1
            try:
                sf_file_matching = [tmp_sf_path for tmp_sf_path in sfList if "_" + fnum + "_SF_PP.mat" in tmp_sf_path][0]
                sf_file_fnum = (sf_file_matching.split(".")[-2]).split("_")[-3]
                
                mat_file_matching = [tmp_mat_path for tmp_mat_path in matList if "_" + fnum + ".mat" in tmp_mat_path][0]
                mat_file_fnum = (mat_file_matching.split(".")[-2]).split("_")[-1]
                
                if self.type == "CN7":
                    raw_file_matching = [tmp_mat_path for tmp_mat_path in rawDataList if "_" + fnum + ".bag" in tmp_mat_path][0]
                elif self.type == "RG3" :
                    raw_file_matching = [tmp_mat_path for tmp_mat_path in rawDataList if "_" + fnum + ".mf4" in tmp_mat_path][0]
                else :
                    raise("CAR TYPE ERROR")
                raw_file_fnum = (mat_file_matching.split(".")[-2]).split("_")[-1]
            except:
                continue
            
            if (fnum == sf_file_fnum and fnum == mat_file_fnum) == False :
                continue
            
            # man_fnum = (os.listdir(self.manDir)[0].split("\\")[-1]).split("_")[3][:3]
        
        # for num in tqdm(range(np.size(sfList))):                
        #     fnum = str(num+1).zfill(3)
            # except_list = []
            # if num+1 in except_list:
            #     continue
            GPS_STATUS = 0
            CHASSIS_STATUS = 0
            MOBILEYE_STATUS = 0
            FRONT_RADAR_STATUS = 0
            CORNER_RADAR_STATUS = 0
            LIDAR_STATUS = 0
            ODD_STATUS = 0
            CSS_STATUS = [] # (0: Normal, 1: GPS inappropriate, 2: Chassis inappropriate, 3: Mobileye data inappropriate, 4: Front Radar inappropriate, 5: Corner Radar inappropriate, 6: Lidar inappropriate 7: OUT of ODD)
            FRAMESIZE = 0
            TYPE = self.type
            DATE = self.date
            sfName = sf_file_matching.split('\\')[-1]
            matName = mat_file_matching.split('\\')[-1]
            rawDataName = raw_file_matching.split('\\')[-1]

            matSf = scipy.io.loadmat(sf_file_matching)
            if self.type == "RG3":
                mat = scipy.io.loadmat(matList[num])
            if self.type == "CN7":
                mat = mat73.loadmat(matList[num])
                mat = h5py.File(matList[num])
            # GPS_STATUS,CHASSIS_STATUS,MOBILEYE_STATUS,FRONT_RADAR_STATUS,CORNER_RADAR_STATUS,LIDAR_STATUS,ODD_STATUS = self.CheckData(self.type,GPS_STATUS,CHASSIS_STATUS,MOBILEYE_STATUS,FRONT_RADAR_STATUS,CORNER_RADAR_STATUS,LIDAR_STATUS,ODD_STATUS,mat)        
            FRAMESIZE,CHASSIS_STATUS = self.GetVehicleFrameSize(self.type, mat)
            try:
                registrationFileRoad = pd.read_excel(self.rgDir +r'\Raw\Registration_' + self.type + r'_' + self.date + r'.xlsx',sheet_name = "road")
                registrationFileWrong = pd.read_excel(self.rgDir +r'\Raw\Registration_' + self.type + r'_' + self.date + r'.xlsx',sheet_name = "road")
            except:
                raise Exception("Registration.xlsx가 없습니다.")
            
            for tmpidx in range(registrationFileWrong.shape[0]):
                if int(fnum) == registrationFileWrong['dataNum'].iloc[tmpidx]:
                    CSS_STATUS = str(registrationFileWrong['Description'].iloc[tmpidx])

                
            adminDirectoryRaw = rawDataDir + '\\' + rawDataName
            adminDirectoryExported = self.matDir + '\\' + matName
            adminDirectoryRegistration = self.rgDir + '\\Raw\\Registration_' + self.type + '_' + self.date +'.xlsx' 
            time = datetime.fromtimestamp(os.path.getctime(adminDirectoryRaw)).strftime('%Y-%m-%d %H:%M:%S')[-8:]  
            adminDate = "20"+self.date[4:6] +"-"+ self.date[0:2] + "-" + self.date[2:4] +"T" + time
            adminTravelTime = FRAMESIZE*float(self.adminSampleTime)/3600
            adminDirectoryPerceptionLDT = " "
            if self.type == 'CN7' :
                adminDirectoryPerceptionLDT = self.matDir + r'\Perception\LDT\CN7_' +self.date + '_'+ fnum + r'_Lidar_PP.mat'
            adminDirectoryPerceptionSF = self.sfDir + '\\' + self.type + '_' +self.date + '_' + fnum + r'_SF_PP.mat'
            adminDirectoryPerceptionRecognition = self.matDir + r'\Perception\Recognition\Recognition_' + self.type + '_' +self.date + '_' + fnum + r'.xlsx'
            adminDirectoryDecisionManeuver = self.matDir + r'\Decision\Maneuver\Manuever_' + self.type + '_' +self.date + '_' + fnum + r'.xlsx'
            adminFileSize = self.ConverSize(os.path.getsize(adminDirectoryRaw))
            adminGeoreferenceCoordinates,GPS_STATUS = self.GetGeoCoordinates(self.type , mat)
            adminTravelDistance ,CHASSIS_STATUS= self.GetTravelDistance(self.type, mat, float(self.adminSampleTime))
            adminAnnotationType = 'auto'
            #####
            
            driver = {
                "sex":"",
                "age":"",
                "experience":""
            }
            
            
            directory = {
                    "raw":  adminDirectoryRaw.replace("\\\\192.168.75.251","D:\\Shares"),
                    "exported": adminDirectoryExported.replace("\\\\192.168.75.251","D:\\Shares"),
                    "registration":adminDirectoryRegistration.replace("\\\\192.168.75.251","D:\\Shares"),
                    "perception":{
                        "LDT":adminDirectoryPerceptionLDT.replace("\\\\192.168.75.251","D:\\Shares"),
                        "SF":adminDirectoryPerceptionSF.replace("\\\\192.168.75.251","D:\\Shares"),
                        "recognition":adminDirectoryPerceptionRecognition.replace("\\\\192.168.75.251","D:\\Shares")            
                    },
                    "decision":{
                        "maneuver": adminDirectoryDecisionManeuver.replace("\\\\192.168.75.251","D:\\Shares")      
                    }
                }
            #####
            


            lane_width, curvature,MOBILEYE_STATUS = self.GetLaneInfo(self.type,mat)
            if np.size(lane_width) == 0 or np.size(curvature) == 0:
                sceneryLaneWidthMax = float(0.0)
                sceneryLaneWidthMin = np.abs(float(0.0))
                sceneryCurvatureMax = float(0.0)
                sceneryCurvaturemin = float(0.0)
            else :
                sceneryLaneWidthMax = str(round(np.max(lane_width),2))
                sceneryLaneWidthMin = str(np.abs(round(np.min(lane_width),2)))
                sceneryCurvatureMax = str(round(np.max(curvature),8))
                sceneryCurvaturemin = str(round(np.min(curvature),8))

            curveFlag, curveFlagSize = self.CalculateCurveEvent(self.type,matSf)
            sceneryEvent = []
            sceneryEvent = pd.DataFrame(columns=['frameIndex', 'roadGeometry'])
            sceneryEvent = self.get_scenery_event(curveFlag , sceneryEvent)
            sceneryRoadName = self.GetRoadName(registrationFileRoad,num)
            # maneuverFile = self.manDir + '\\' + "Maneuver_" + self.type + '_' +self.date + '_' +fnum + '.xlsx' ################### 여기 는 나중에 바뀔 내용
            # maneuverFile = os.getcwd() + '\\Output_xlsx\\'+self.type +'_' + self.date+'\\Annotation_'+self.type+'_'+self.date+'_' + fnum +'.xlsx'
            maneuverFile = os.getcwd() + '\\Output_xlsx\\'+self.type +'_' + self.date+'\\Annotation_'+self.type+'_'+self.date+'_' + fnum +'.xlsx'
            try:
                maneuverLabel = pd.read_excel(maneuverFile, sheet_name = 'Label')
            except:
                try:
                    maneuverLabel = pd.read_excel(maneuverFile)
                except:
                    continue
            label = maneuverLabel

            # sceneryRoadLabel = label.iloc[:,6:8].query('RoadFrameIndex >= 0')
            # sceneryRoadLabelNum =  sceneryRoadLabel.shape[0]   


            illumination = int(time[:2])
            if (illumination >= 6 and illumination < 18):
                illumination = "Day"
            else:
                illumination = "Night"
            environmentIllumination = ["illumination" , illumination]
            environmentWeather = ["weather" , " "]              


            dynamicInitEgoVelocity = (np.array((matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][0,7])) +np.array((matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][0,6])))/2
            dynamicInitRelVelX = np.array((matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][25,0,0]))      
            maneuverLabelDynamic = label.iloc[:,0:5]    
            indexSize = maneuverLabelDynamic.dropna().shape[0]
            columnSize = maneuverLabelDynamic.dropna().shape[1]
            initCount = int(np.size(maneuverLabelDynamic.query('FrameIndex == 1'))/columnSize)

            tmpEgoVelocity = 0
            tmpRelvelX = 0
            last_frameIndex = int(label['FrameIndex'].iloc[indexSize-1])
            longitudinalActionVelocity = np.zeros(indexSize)


            for frameIndex in range(indexSize):
                frameNum = int(label['FrameIndex'].iloc[frameIndex]) -1 ## python 은 -1 되어서 사용해야 하므로 전처리
                try:
                    tmpEgoVelocity = (float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 6]) + float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 7]))/2
                    for trackIdx in range(self.TRACKNUM):
                        if int(matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][self.FT_ID,trackIdx,frameNum]) == int(label['ID'].iloc[0]):  # int(label['ID'].iloc[0]) 을 하는이유 Ego 의 아이디와 비교해서 확인해야 하기 때문에
                            tmpRelvelX = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][self.FT_RELX,trackIdx,frameNum]
                    longitudinalActionVelocity[frameIndex] =float(tmpEgoVelocity + tmpRelvelX)
                except:
                    continue
            longitudinalActionAcceleration=" "
            lateralActionAcceleration=" "
            lateralActionVelocity=" "     
            #########################################################################################################################
            #participant
            #########################################################################################################################
            participant = []
            for i in range(indexSize):
                # participant_objframe['participants'].append(participants)
                participants = self.GetParticipants(matSf,i,maneuverLabelDynamic,FRAMESIZE)
                
                participant.append(self.GetParticipantObjframe(i,participants,maneuverLabelDynamic))      
            numInitLane = ""
            numMaxLane =""
            scenery = {
                "roadName":sceneryRoadName,
                "event":[],
                "laneWidthMax":sceneryLaneWidthMax,
                "laneWidthMin":sceneryLaneWidthMin,
                "curvatureMax":sceneryCurvatureMax ,
                "curvatureMin":sceneryCurvaturemin,
                "numInitLane":numInitLane,
                "numMaxLane":numMaxLane
            }
            curveFlagSize
    
            # if mode == AUTOCALCULATIONMODE :
            for i in range(curveFlagSize):
                scenery['event'].append({"frameIndex":int(sceneryEvent['frameIndex'].iloc[i]),"roadGeometry":sceneryEvent['roadGeometry'].iloc[i]})
            # elif mode == ANNOTATIONMODE  :
            #     for i in range(sceneryRoadLabelNum):
            #         scenery['event'].append({"frameIndex":int(sceneryRoadLabel['RoadFrameIndex'].iloc[i]),"roadGeometry":sceneryRoadLabel['roadGeometry'].iloc[i]})
        
            # #이하 기능은 annotation이 2줄 이상 있을시에는 annotation을 이용하고 그 외에는 자동으로 생성하는 것을 사용하는 기능입니다. 
            # elif mode == MIXMODE  :
            #     if sceneryRoadLabelNum == 1:
            #         for i in range(curveFlagSize):
            #             scenery['event'].append({"frameIndex":int(sceneryEvent['frameIndex'].iloc[i]),"roadGeometry":sceneryEvent['roadGeometry'].iloc[i]})
            #     else :
            #         for i in range(sceneryRoadLabelNum):
            #             scenery['event'].append({"frameIndex":int(sceneryRoadLabel['RoadFrameIndex'].iloc[i]),"roadGeometry":sceneryRoadLabel['roadGeometry'].iloc[i]})
            # else :
            #     print("select correct mode!!")     
    
            environment = {
                    environmentIllumination[0]:environmentIllumination[1],
                    environmentWeather[0]:environmentWeather[1]
            }        


            dynamic = {
                "init":[],
                "story":{
                    "event":[]
                }
            }          
            
            dynamic_init_action_ego = { # for init 
                "longitudinalAction":{     
                    "velocity":float(longitudinalActionVelocity[0]), # ego 의 차속
                    "acceleration":longitudinalActionAcceleration
                                },
                "lateralAction":{
                    "acceleration":lateralActionAcceleration,
                    "velocity":lateralActionVelocity
                } 
            }

            dynamic_init_action = { # for init 이지만 ego 가 아닌 대상을 위해
                "longitudinalAction":{     
                    "velocity":" ",
                    "acceleration":longitudinalActionAcceleration
                                },
                "lateralAction":{
                    "acceleration":lateralActionAcceleration,
                    "velocity":lateralActionVelocity
                } 
            }  

            for i in range(initCount):
                if i ==0:
                    dynamic['init'].append({"frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),\
                    "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),"recognition":str(maneuverLabelDynamic['Recognition'].iloc[i])\
                    ,"maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i]),"category":int(maneuverLabelDynamic['Category'].iloc[i]),\
                    "action": dynamic_init_action_ego})
                else :
                    dynamic['init'].append({"frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),\
                    "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),"recognition":str(maneuverLabelDynamic['Recognition'].iloc[i])\
                    ,"maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i]),"category":int(maneuverLabelDynamic['Category'].iloc[i]),\
                    "action": dynamic_init_action})                   

            dynamic_story_startTrigger = {
                "conditionName":" ",
                "participantID":" "
            }

            for i in range(initCount , indexSize): # init 이 아닌 부분을 전부 반복문으로 실행
                dynamic_event_key ={
                    "frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),
                    "startTrigger":dynamic_story_startTrigger,
                    "actors":{
                        "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),
                        "category":int(maneuverLabelDynamic['Category'].iloc[i]),
                        "recognition":str(maneuverLabelDynamic['Recognition'].iloc[i]),
                        "maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i])
                    },
                    "action":{
                        "longitudinalAction":{     
                            "velocity":longitudinalActionVelocity[i],
                            "acceleration":longitudinalActionAcceleration
                                        },
                        "lateralAction":{
                            "acceleration":lateralActionAcceleration,
                            "velocity":lateralActionVelocity
                        }                     
                    }
                }
                dynamic['story']['event'].append(dynamic_event_key)

            ### Generate CSS ###

            CSS_STATUS = self.CheckCSSStatus(GPS_STATUS,CHASSIS_STATUS,MOBILEYE_STATUS,FRONT_RADAR_STATUS,CORNER_RADAR_STATUS,LIDAR_STATUS,ODD_STATUS,CSS_STATUS = CSS_STATUS)
            adminStatus = CSS_STATUS
            CSS_STATUS = []        
            
            CSS = self.GetCSS(self.adminDataType,self.adminSampleTime,self.adminVersion,self.adminProjectName,directory,driver,\
                adminDate,adminTravelTime,adminFileSize,self.GeoreferenceType,adminGeoreferenceCoordinates,self.adminCMGT,\
                self.adminAESGT,adminStatus,adminTravelDistance,adminAnnotationType,scenery,environment,dynamic,participant)      
            # /print(CSS) 
            jsonFilePath = self.jsonDir +"\\"+ TYPE + '_' + DATE + "_AutoCuration"
            jsonFileName = str(num+1).zfill(3) + r'_AutoCuration_temp.json'
            if os.path.isdir(self.jsonDir) == False:
                os.mkdir(self.jsonDir)
            if os.path.isdir(jsonFilePath) == False:
                os.mkdir(jsonFilePath)            
            with open(jsonFilePath+jsonFileName, 'w') as outfile:
                json.dump(CSS, outfile,indent='\t')    

        jsonList = natsort.natsorted(glob.glob(self.jsonDir + '\\*_AutoCuration_temp.json'))
        temp = []
        for i in range(np.size(jsonList)):
            with open(jsonList[i]) as file:
                data = json.load(file)
                temp.append(data)
        realTime = datetime.now()
        realTime = '_'+str(realTime)[:4] +'_'+str(realTime)[5:7] +'_'+ str(realTime)[8:10] +'_'+ str(realTime)[11:13]+'_'+str(realTime)[14:16]
        

        temp_num = 0
        with open(jsonFilePath + '\\' + TYPE+ '_' + DATE+'_'+realTime+'_AutoCuration.json' ,"w") as new_file:
            # json.dump(temp , new_file,indent='\t')
            json.dump([temp[i][0] for i in range(np.size(jsonList))] , new_file,indent='\t')
        
        
        for file in jsonList:
            if file.endswith('_AutoCuration_temp.json'):
                os.remove(file)
        print("########## CODE " +self.type + "_" + "AUTOCURATION END   ##########")

if __name__ == "__main__":
    dir = ""
    