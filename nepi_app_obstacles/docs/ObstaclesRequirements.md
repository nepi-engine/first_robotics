# **SVX API Requirements**

Main assumptions/design choices

1. One SVX instance \= one servo. A turret is two instances. Coordinating them is the app's job.  
2. Open loop. A 3-wire PWM servo reports nothing back. We only know what we last commanded.  
3. API speaks degrees. Conversion to pulse width is a driver concern, not an API concern.

# **Hardware Connection**

Since the NEPI device is enclosed and its GPIO is not accessible, only USB is exposed. USB cannot carry a PWM signal directly, so a board must sit between the NEPI device and the servos to generate the pulse. There are two options, first is the Pololu Maestro which is a built USB servo controller with factory firmware already on it. The second option is the ESP32 which I have currently at home, which would need custom firmware written for it. 

The issue is that the Maestro Mini is more expensive at around 30-35 dollars but already provides a good firmware for plug and play use. Additionally, this is the cheapest option available which already provides some basic firmware out of the box. Cheaper options like the esp32 or some other sort of simple microcontroller would need to have firmware created.

# **API Conversion**

Since this api will focus on controlling a single servo, blind to the fact that it is, we do not need to have separate settings for both instead just combine into one. Additionally, since this is a positional not continuous servo, we need to have absolute positioning and and also software limits. Additionally a useful parameter for gain in the turret we could control would be the gain which is the speed/speed ratio. The final kept items are in Figure 1 and attached python file, while items cut out are in Figure 2\. 

**Figure 1: Kept API Parameters** 

| def \_\_init\_\_(self, device\_info,              capSettings, factorySettings,              settingUpdateFunction, getSettingsFunction,              factoryControls,              factoryLimits \= None,              data\_source\_description \= 'servo',              data\_ref\_description \= 'servo\_axis',              stopMovingCb \= None,              gotoPositionCb \= None,              getPositionCb \= None,          \# Returns last sent position.              setSoftLimitsCb \= None,     \# Software defined limits              getSoftLimitsCb \= None,                            setSpinDirection \= None,              getSpinDirection \= None,              setSpeedRatioCb \= None,        \# None \=  No speed adjustment; speed ratio arg              getSpeedRatioCb \= None,        \# None \= No speed adjustment; returns speed ratio              setSpeedMaxCb \= None,              getSpeedMaxCb \= None,              goHomeCb \= None,               \# None \==\> No homing              setHomePositionCb \= None,              setHomePositionHereCb \= None,              deviceResetCb \= None,              log\_name \= None,              log\_name\_list \= \[\],              msg\_if \= None             ): |
| :---- |

**Figure 2: Cut API Parameters** 

| movePanCb \= None,           moveTiltCb \= None, movePanSpeedRatioCb \= None, moveTiltSpeedRatioCb \= None, gotoPanPositionCb \= None,       \# collapse into gotoPositionCb gotoTiltPositionCb \= None, setPanSpeedRatioCb \= None,      \# collapse into setSpeedRatioCb setTiltSpeedRatioCb \= None, getPanSpeedRatioCb \= None,      \# collapse into getSpeedRatioCb getTiltSpeedRatioCb \= None, getPositionTimesCb \= None,      \# only fed the speed-smoothing math, now dead calibrateCenterCB \= None,       \# a bare PWM servo has no hardstops to find getNavPoseCb \= None,            \# NavPose is navigation, out of scope navpose\_update\_rate \= 10, |
| :---- |

# 

# **Status Message**

Position is reported as the last commanded value, since an open-loop servo returns nothing. Speed is reported as both a ratio and a max, so the UI can show "0.5 (= 30°/s)". The soft limits are included so the UI knows the bounds of the position slider. The final kept fields are in Figure 3\.

**Figure 3: Kept Status Message Fields**

| \#\# SVX Servo status msg \#\# One SVX device \= one servo \# Device identity \- displayed in UI, used to distinguish devices string device\_name string device\_path string device\_node\_name string serial\_num string hw\_version string sw\_version string data\_source\_description string data\_ref\_description \# Where to find this device's other interfaces string settings\_topic string save\_data\_topic \# Which controls the UI should draw bool has\_absolute\_positioning bool has\_adjustable\_speed bool has\_limit\_controls bool has\_homing bool has\_set\_home bool has\_spin \# Rounding for commands sent and values displayed int32 move\_decimal\_place int32 report\_decimal\_place \# Where the servo is and where it was told to go. \# Reported as last commanded value \- open loop, nothing is measured. float32 position\_now\_deg float32 position\_goal\_deg float32 position\_now\_ratio float32 position\_goal\_ratio \# Bounds of the position slider in the UI, and the range the IF clamps to float32 min\_softstop\_deg float32 max\_softstop\_deg \# Whether the axis is inverted bool reverse\_enabled \# Spin direction int32 spin\_direction bool is\_spinning \# speed\_ratio is the 0.0-1.0 dial the user sets. \# speed\_max\_dps is what 1.0 means in deg/sec, so the UI can show real units. float32 speed\_max\_dps float32 speed\_ratio float32 home\_pos\_deg \# Surfaces driver faults to the UI string\[\] error\_msgs |
| :---- |

# **Capabilities Service**

Final List of kept capabilities is below in figure 4\.

**Figure 4: Kept Capabilities List** 

| string device\_name string device\_path string device\_node\_name uint8 mode bool has\_absolute\_positioning   \# set by gotoPositionCb bool has\_adjustable\_speed          \# set by setSpeedRatioCb \+ getSpeedRatioCb bool has\_limit\_control                 \# set by getSoftLimitsCb bool has\_homing                               \# set by goHomeCb bool has\_set\_home                          \# set by setHomePositionHereCb bool has\_spin\_control                  \# if servo is continuous bool has\_goto\_control bool has\_stop\_control float32 max\_speed\_dps |
| :---- |

# 

# **Controls**

The capabilities service says what the servo can do; the status message says what it is doing and the controls are how you command it. Each control is a topic the IF subscribes to on receipt it clamps against the soft limits and calls the matching callback. 

**Figure 5:** 

| goto\_position                                     float32 (deg)      goto\_ratio                                            float32 (0.0-1.0) set\_soft\_limits                                   ServoLimits        set\_speed\_ratio                                float32 (0.0-1.0) set\_speed\_max\_dps                      float32 (deg/s)  set\_reverse\_enable                         bool  stop\_moving                                      Empty go\_home                                              Empty  set\_home\_position                         float32 (deg) set\_home\_position\_here            Empty  reset\_device                                       Empty set\_spin\_direction                           int32 |
| :---- |

