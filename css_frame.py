'''
CSS frame 
version 0.0.1
numMaxLane => numOfLane 으로 변경 
iso 21448, ASAM open label 의 표현법을 따름.
기존 numMaxLane = int 로만 존재 numMinLane 없었음
numOfLane = [int, int] 로 변경 openlabel의 표현법 따름.

'''
CSS = {
    ### Admin ###
    "directory":{
        "raw":"",},    
    "date":"",
    "dataType":"",
    "travelTime":"",
    "travelDistance":"",
    "fileSize":"",
    "georeference":{
        "type":"",
        "coordinate":[],
        },
    "sampleTime":"",
    
    ### Scenery ###
    "scenery":{
        "roadName":"",
        "event":[],
        "laneWidthMax":"",
        "laneWidthMin":"",
        "curvatureMax":"",
        "curvatureMin":"",
        "numOfLane":[],
    },
    
    ### environment ###
    "environment":{
        "illumination":"",
        "weather":"",
        },
    
    ### dynamic ###
    "dynamic":{
        "init":[],
        "story":{
            "event":[],
            },
        },
    
    ### participant ### 
    "participant":[],
}


scenery_event = {
    "frameIndex":"",
    "roadGeometry":""
    }

dynamic_init = {
    "frameIndex":"",
    "participantID":"",
    "category":"",
    "recognition":"",
    "maneuver":"",
    "action":{
        "longitudinalAction":{
            "position":"",
            "acceleration":"",
            "velocity":"",
        },
        "lateralAction":{
            "position":"",
            "acceleration":"",
            "velocity":"",
            },
        },
    }

dynamic_story_event = {
    "frameIndex":"",
    "actors":{
        "participantID":"",
        "category":"",
        "recognition":"",
        "maneuver":"",
    },
    "action":{
        "longitudinalAction":{
            "position":"",
            "acceleration":"",
            "velocity":"",
            },
        "lateralAction":{
            "position":"",
            "acceleration":"",
            "velocity":"",  
            },
        },
}

particiapnt = {
    "frameIndex":"",
    "ID":"",
    "participants":[],
}

participants = {
    "recognition":"",
    "maneuver":"",
    "category":"",
    "participantID":"",
}



'''
version 0.0.0 
'''
# CSS = {
#     ### Admin ###
#     "directory":{
#         "raw":"",},    
#     "date":"",
#     "dataType":"",
#     "travelTime":"",
#     "travelDistance":"",
#     "fileSize":"",
#     "georeference":{
#         "type":"",
#         "coordinate":[],
#         },
#     "sampleTime":"",
    
#     ### Scenery ###
#     "scenery":{
#         "roadName":"",
#         "event":[],
#         "laneWidthMax":"",
#         "laneWidthMin":"",
#         "curvatureMax":"",
#         "curvatureMin":"",
#         "numOfLane":"",
#     },
    
#     ### environment ###
#     "environment":{
#         "illumination":"",
#         "weather":"",
#         },
    
#     ### dynamic ###
#     "dynamic":{
#         "init":[],
#         "story":{
#             "event":[],
#             },
#         },
    
#     ### participant ### 
#     "participant":[],
# }


# scenery_event = {
#     "frameIndex":"",
#     "roadGeometry":""
#     }

# dynamic_init = {
#     "frameIndex":"",
#     "participantID":"",
#     "category":"",
#     "recognition":"",
#     "maneuver":"",
#     "action":{
#         "longitudinalAction":{
#             "position":"",
#             "acceleration":"",
#             "velocity":"",
#         },
#         "lateralAction":{
#             "position":"",
#             "acceleration":"",
#             "velocity":"",
#             },
#         },
#     }

# dynamic_story_event = {
#     "frameIndex":"",
#     "actors":{
#         "participantID":"",
#         "category":"",
#         "recognition":"",
#         "maneuver":"",
#     },
#     "action":{
#         "longitudinalAction":{
#             "position":"",
#             "acceleration":"",
#             "velocity":"",
#             },
#         "lateralAction":{
#             "position":"",
#             "acceleration":"",
#             "velocity":"",  
#             },
#         },
# }

# particiapnt = {
#     "frameIndex":"",
#     "ID":"",
#     "participants":[],
# }

# participants = {
#     "recognition":"",
#     "maneuver":"",
#     "category":"",
#     "participantID":"",
# }