

__author__ = 'lake'
__revision__ = 0.1
__date__ = "$Date$"[7:-2]

''' ====================== To be removed ======================  '''
import memojito
import pyglet
from operator import attrgetter
getX = attrgetter('x')
getY = attrgetter('y')
getR = attrgetter('rotation')
pyglet.resource.reindex()
from cocos.director import director
from cocos.scene import Scene
from cocos.actions import FadeIn
from cocos.layer import ScrollableLayer, ScrollingManager
from rabbyt.collisions import collide_single
from steering import Steerable
from math import radians, degrees, sin, cos, pi, atan2
pi_2 = pi*2.0
pi_1_2 = pi/2.0
pi_1_4 = pi/4.0
pi_3_4 = (pi*3)/4
''' ======================  To be removed ======================  '''




import roslib
import rospy
import numpy
import math
import os
import subprocess
import re
import time
import tf
import socket
import std_msgs
import threading

# Jamie's API
from hri_api.entities import Person, World, Saliency
from zoidstein_hri.zoidstein import Zoidstein, ZoidExpression, ZoidGestureData
from hri_api.query import Query

import sys
import random

import owyl

from numpy import linalg

## Owyl provides the wisdom
from owyl import blackboard

from geometry_msgs.msg import Point
from simple_face_tracker.msg import targets
from std_msgs.msg import String, Bool
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        yield self.match
        raise StopIteration

    def match(self, *args):
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class Tree:
    def __init__(self, tree_name):
        self.blackboard = blackboard.Blackboard()

        # self.BehaviorNode = rospy.init_node("behavior_tree")
        rospy.Subscriber("itf_listen", String, self.audioInputCallback)
        rospy.Subscriber("zenodial_talk", String, self.zenoDialCallback)
        rospy.Subscriber("speech_active", Bool, self.isSpeakingCallback)
        rospy.Subscriber("facedetect", targets, self.faceDetectCallback)
        rospy.Subscriber("/nmpt_saliency_point", targets, self.saliencyCallback)
        self.itf_talk_pub = rospy.Publisher("itf_talk", String, queue_size=1)
        self.zenodial_listen_pub = rospy.Publisher("zenodial_listen", String, queue_size=1)
        self.robot_movement_pub = rospy.Publisher("robot_movement", String, queue_size=1)

        self.blackboard["commandKeywords"] = {
                'Walk Forward': ['forward', 'ahead', 'straight', 'forwards'],
                'Walk Backward': ['back', 'backward', 'back up', 'backwards'],
                'Turn Left': ['turn left', 'turn lefts', 'turns left'],
                'Turn Right': ['turn right', 'turn rights', 'turns right']}

        ### Inputs
        self.blackboard["faceTarget"] = {}                        # seq no: [Person obj, x, y, velocity, age, disappear_age]
        self.blackboard["saliencyTarget"] = {}                    # seq no: [Person obj, x, y, velocity, age, disappear_age]
        # self.saliencyTargetPos = [0.0, 0.0]         # position of the current saliency target
        # self.saliencyTargetVel = 0.0                # average velocity of the current saliency target over the last second
        # self.saliencyTargetAge = 0                  # time since the last significant change in the saliency target position
        # self.faceTargetPos = [[0.0, 0.0]]           # position of the current face target
        # self.faceTargetVel = 0.0                    # average velocity of the current face target over the last second
        # self.faceTargetAge = 0                      # time since the last significant change in the face target position
        # self.bodyTargetPos = [[0.0, 0.0]]           # position of the current body target
        # self.bodyTargetVel = 0.0                    # average velocity of the current body target over the last second
        # self.bodyTargetAge = 0                      # time since the last significant change in the body target position
        self.blackboard["audioInput"] = ""                        # string output of speech-to-text algorithm, raw form
        self.blackboard["audioInputAge"] = 0                      # time since the last significant parse of the audio input
        self.blackboard["audioInputVol"] = 0                      # average volume or magnitude of the last audio input
        self.blackboard["rosInput"] = ""                          # string representation of miscellaneous commands from other ros components, usually blender
        self.blackboard["rosInputAge"] = 0                        # time since the last ros command
        self.blackboard["emotionInput"] = ""                      # string output of the audio-emotion-analysis algorithm
        self.blackboard["emotionInputAge"] = 0                    # time since the last significant chance in emotional state
        self.blackboard["speechOutput"] = ""                      # string representation of the last speech output from the robot
        self.blackboard["speechOutputAge"] = 0                    # time since the last speech output from the robot
        # self.blackboard["animationOutput"] = ""                   # string representation of the last animation output from the robot
        # self.blackboard["animationOutputAge"] = 0                 # time since the last animation output from the robot
        # self.blackboard["animationOutputDur"] = 0                 # for zeno body paint
        self.blackboard["randomInput"] = 0                        # a random percentile for random behaviors
        self.blackboard["randomInput2.5"] = self.blackboard["randomInput"] * 2.5

        ### Globals
        self.blackboard["blinkChance"] = 0.011    # @ 60 fps a 1.1% chance to start a blink each frame should give us a nice frequency
        self.blackboard["blinkChance1.5"] = self.blackboard["blinkChance"] * 1.5
        self.blackboard["blinkChance1.2"] = self.blackboard["blinkChance"] * 1.2
        self.blackboard["highBodyVel"] = 1        # Not sure what would be considered a high velocity for the body - use 1 for now
        self.blackboard["eyeFreedom"] = 0.5
        self.blackboard["neckFreedom"] = 0.5
        self.blackboard["Idle"] = "Idle"
        self.blackboard["StopSpeech"] = "Stop Speech"
        self.blackboard["WalkForward"] = "Walk Forward"
        self.blackboard["WalkBackward"] = "Walk Backward"
        self.blackboard["TurnLeft"] = "Turn Left"
        self.blackboard["TurnRight"] = "Turn Right"
        self.blackboard["PointUp"] = "Point Up"
        self.blackboard["PointDown"] = "Point Down"
        self.blackboard["LookUp"] = "Look Up"
        self.blackboard["LookDown"] = "Look Down"
        self.blackboard["Wave"] = "Wave"
        self.blackboard["Smile"] = "Smile"
        self.blackboard["Frown"] = "Frown"
        self.blackboard["FrownMouth"] = "Frown Mouth"
        self.blackboard["OpenMouth"] = "Open Mouth"
        self.blackboard["eyeFree0.25"] = self.blackboard["randomInput"] * 0.25 * self.blackboard["eyeFreedom"]
        self.blackboard["eyeFree0.75"] = self.blackboard["randomInput"] * 0.75 * self.blackboard["eyeFreedom"]
        self.blackboard["neckFree0.1"] = self.blackboard["randomInput"] * 0.1 * self.blackboard["neckFreedom"]
        self.blackboard["neckFree0.3"] = self.blackboard["randomInput"] * 0.3 * self.blackboard["neckFreedom"]
        self.blackboard["boolean_true"] = True
        self.blackboard["boolean_false"] = False

        ### Locals
        self.blackboard["commandName"] = ""
        self.blackboard["commandInput"] = ""
        self.blackboard["actionName"] = ""
        self.blackboard["bodyOrFace"] = ""
        self.blackboard["targetPos"] = ""
        self.blackboard["glanceOrSaccadeTargetPos"] = {}
        self.blackboard["firstGreeting"] = False
        self.blackboard["speechActive"] = False
        self.blackboard["robotName"] = ""


        ### Subtrees
        ## Blink Subtree. A small example of a tree to run in parallel with the other subtrees.
        # Assumes randomInput is recalculated each frame
        self.blinkSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(
                        owyl.sequence(
                            self.isSwitchingTarget(),
                            self.isLess(num1="randomInput", num2="blinkChance1.5"),  # blink 50% more often when switching targets
                            self.blink()
                        ),
                        owyl.sequence(
                            owyl.selector(
                                # self.isGreater(num1=linalg.norm(self.bodyTargetVel), num2=1),
                                self.isSpeaking()
                            ),
                            self.isLess(num1="randomInput", num2="blinkChance1.2"),
                            self.blink()
                        ),
                        owyl.sequence(
                            self.isLess(num1="randomInput", num2="blinkChance"),
                            self.blink()
                        )
                    )
                ),
                # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                limit_period=0.05
            )

        ## Announce the action we're about to take and then reset the robot to a default stance.
        # Though we announce an action, this tree doesn't execute the action.
        # Assumes actionName has been set
        self.announceAndResetTree = \
            owyl.sequence(  # announce the action and then reset
                owyl.selector(  # If we're not speaking, then speak
                    self.isSpeaking(),
                    self.actionToPhrase(key="actionName"),
                    self.sayStartAction(key="actionPhrase")
                ),
                owyl.selector(  # If we're no in a default stance, reset (blend) to the default stance
                    self.isDefaultStance(),
                    self.resetToDefaultStance()
                )
            )

        ## Executes a basic command, such as to play an animation.
        # Assumes commandName has been set
        # Assumes commandInput has been set
        # Assumes bodyOrFace has been set
        # Will announce the command (if not already speaking)
        # before showing the associated animation
        self.executeBasicCommandSubtree = \
            owyl.sequence(
                self.actionToPhrase(key="commandInput"),
                self.isCommandPhrase(commandName="commandName", actionPhrase="actionPhrase"),
                self.setVariable(var="actionName", value="commandName"),
                owyl.selector(  # try the command sequence (subtree) or report failure
                    owyl.sequence(
                        owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                        self.showAction(action="actionName")
                        # self.showAction(action="actionName", part="bodyOrFace"),  # Finally play the command's animation
                    ),
                    owyl.sequence(
                        self.say(utterance="I'm sorry, Dave, I'm afraid I can't do that..."),
                        owyl.fail()
                    )
                )
            )

        ## Select a basic command to execute, once we know that we've been given a command.
        # Assumes bodyOrFace has been set, to distinguish body actions from face actions
        # Assumes commandInput has been set to either audioInput or rosInput
        self.selectBasicCommandSubtree = \
            owyl.selector(  # Select from one of several mutually exclusive behaviors
                owyl.sequence(  # If we should be idling, then try to play the Idle animation...
                    self.setVariable(var="commandName", value="Idle"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="StopSpeech"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we're commanded to anc can walk to target location, then play the walk animation until we reach the target
                    self.setVariable(var="commandName", value="WalkForward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="WalkBackward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="TurnLeft"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="TurnRight"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="PointUp"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="PointDown"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="LookUp"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="LookDown"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="Wave"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we should show an emotion, then select the right one and show it.
                    self.setVariable(var="commandName", value="Smile"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="Frown"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="FrownMouth"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var="commandName", value="OpenMouth"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                )
            )

        ## Tracks the target face or salient point
        # Assumes targetPos has been set to face, body, or salient position
        # self.faceGaze = \
        #     owyl.sequence(
        #         # TODO: Get clarification from Hanson and others on the conditions for tracking
        #         # owyl.selector(
        #         #   self.isFaceNearestAudioSource(self.faceTargetPos), # Do we have the source of the audio?
        #         #   self.isFaceMostSalient(self.faceTargetAge, self.saliencyTargetAge), # Do we know the degree/magnitude of saliency?
        #         #   self.isFaceCentered(self.faceTargetPos), # Can we find the centroid of all the faces?
        #         #   self.isLess(self.randomInput, self.blinkChance*2.0) # Should we really switch tracking targets this often?
        #         # ),
        #
        #         # self.faceTrack(pos=targetPos, eyeFree=eyeFreedom, neckFree=neckFreedom, rand=-1)  # -1 here indicating that we'll track this point until told to stop
        #     )

        ## Displays the surprised emotional expression under certain conditions
        # Assumes targetPos has been set to face, body, or salient position
        self.startle = \
            owyl.sequence(
                owyl.selector(
                    self.isGreater(num1="audioInputVol", num2=1),  # or whatever counts for a high volume
                    # self.isGreater(num1=linalg.norm(self.faceTargetVel), num2=1),
                    # self.isGreater(num1=linalg.norm(self.bodyTargetVel), num2=1),
                    # self.isGreater(num1=linalg.norm(self.saliencyTargetVel), num2=1)
                ),
                self.showAction(action="OpenMouth")
                # self.showAction(action="Surprised", part="face")
            )

        ## Random eye movements which are much faster and less pronounced than glances.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        # Assumes eyeFreedom and neckFreedom have been set to appropriate degrees of freedom
        self.saccade = \
            owyl.selector(
                owyl.sequence(
                    self.isGreater(num1="randomInput", num2=0.5),
                    self.faceTrack(pos="glanceOrSaccadeTargetPos", eyeFree="eyeFree0.25", neckFree="neckFree0.1", rand="randomInput")
                ),
                owyl.sequence(
                    self.faceTrack(pos="glanceOrSaccadeTargetPos", eyeFree="eyeFree0.75", neckFree="neckFree0.3", rand="randomInput")
                )
            )

        ## Random eye movements which signal recognition of targets.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        self.glance = \
            owyl.selector(
                owyl.sequence(
                    owyl.selector(
                        # self.isLess(num1=self.faceTargetAge, num2=1),
                        self.isLess(num1="randomInput", num2=0.0025)
                    ),
                    self.faceTrack(pos="glanceOrSaccadeTargetPos", eyeFree="eyeFreedom", neckFree="neckFreedom", rand="randomInput2.5")
                ),
                owyl.sequence(
                    owyl.selector(
                        self.isLess(num1="saliencyTargetAge", num2=1),
                        self.isLess(num1="randomInput", num2=0.0025)
                    ),
                    self.faceTrack(pos="glanceOrSaccadeTargetPos", eyeFree="eyeFreedom", neckFree="neckFreedom", rand="randomInput2.5")
                )
            )

        ## After tracking at a new target face, ZoidStein will execute a scripted greeting.
        # Be careful not to play this more than once in the same encounter.
        self.greeting = \
            owyl.sequence(
                self.isVariable(var="firstGreeting", value="boolean_false"),
                self.setVariable(var="firstGreeting", value="boolean_true"),
                self.say(utterance="Hello!"),
                self.showAction(action="Wave"),
                self.showAction(action="Smile")
                # self.showAction(action="Wave", part=self.UPPER_BODY),
                # self.showAction(action="Smile", part=self.HEAD_NECK)
            )

        ## When people are too close, move head back and up while playing the afraid expression animation.
        self.awkward = \
            owyl.sequence(
                # self.showAction(action="Afraid", part="face")
                self.showAction(action="LookUp"),
                self.showAction(action="Frown")
                # self.showAction(action="LookUp", part=self.HEAD_NECK),
                # self.showAction(action="Frown", part=self.HEAD_NECK)
            )

        ## When people are very close, move head forward and down while playing the innoscent expression animation.
        self.shy = \
            owyl.sequence(
                # self.showAction(action="Innocent", part=self.HEAD_NECK)
                self.showAction(action="LookDown"),
                self.showAction(action="Frown")
                # self.showAction(action="Look Down", part=self.HEAD_NECK),
                # self.showAction(action="Frown", part=self.HEAD_NECK)
            )

        ## In general, ZoidStein's expressions should mimic the emotional input of its targets.
        self.mimic = \
            owyl.selector(
                owyl.sequence(
                    self.isVariable(var="emotionInput", value="Happy"),
                    # self.showAction(action="Happy", part=self.UPPER_BODY),
                    # self.showAction(action="Happy", part=self.HEAD_NECK)
                )
                # TODO: Do we have to mimic all the emotionInput or just happy?
            )

        """
        Creating the tree
        """
        for case in switch(tree_name):
            if case("BasicZenoTree"):
                self.blackboard["robot"] = Zeno()
                self.blackboard["robotName"] = "Zeno"
                self.tree = self.makeBasicZenoTree()
                # self.do_every(0.01, self.tree.next)
                while True:
                    self.tree.next()
                break
            if case("BasicZoidSteinTree"):
                self.blackboard["robot"] = Zoidstein()
                self.blackboard["robotName"] = "Zoid"
                self.tree = self.makeBasicZoidSteinTree()
                # self.do_every(0.01, self.tree.next)
                while True:
                    self.tree.next()
                break
            if case():
                rospy.loginfo("Unrecognized Tree Name!\n")

    def makeBasicZoidSteinTree(self):
        ## body behavior subtree
        # zoidSteinBodySubtree = \
        # Temp moved to the root tree

        ## face & neck behavior subtree
        # zoidSteinFaceSubtree = \
        # Temp moved to the root tree

        ## ZoidStein's root tree
        zoidSteinTree = \
            owyl.parallel(  # At the highest level, run several parallel behaviors
                # owyl.visit(zoidSteinBodySubtree, blackboard=self.blackboard),
                # owyl.visit(zoidSteinFaceSubtree, blackboard=self.blackboard),

                ######################################## zoidSteinBodySubtree ########################################
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.selector(  # Select response to command or natural behavior
                            owyl.sequence(  # If the last audio or blender input is a command, then select a response
                                self.isCommand(key="audioInput"),
                                # self.setVariable(var="bodyOrFace", value="body"),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            # It's not a command, so start checking for natural behaviors
                            owyl.sequence(
                                self.isNoSalientTarget(),
                                self.isNoFaceTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),
                                # There's nothing to do, so let's dance!
                                # self.dance()
                                # owyl.visit(zoidSteinBodyDance, blackboard=self.blackboard)
                            )
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05
                ),

                ##### zoidSteinFaceSubtree #####
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.selector(  # Select from one of several mutually exclusive face & neck behaviors
                            owyl.sequence(
                                self.isFaceTarget(),
                                self.isNoSalientTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),

                                owyl.visit(self.faceGaze, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isSalientTarget(),
                                self.isNoFaceTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),

                                owyl.visit(self.faceGaze, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                owyl.selector(
                                    self.isAudioInput(),
                                    self.isRosInput()
                                ),
                                owyl.selector(
                                    owyl.sequence(
                                        self.isCommand(key="audioInput"),
                                        self.setVariable(var="commandInput", value="audioInput")
                                    ),
                                    owyl.sequence(
                                        self.isCommand(key="rosInput"),
                                        self.setVariable(var="commandInput", value="rosInput")
                                    ),
                                ),
                                # self.setVariable(var="bodyOrFace", value=self.HEAD_NECK),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isAudioInput(),
                                self.toDialSystem(key="audioInput")
                            )
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05
                ),

                ######################################## General tree ########################################
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            # Poll for input coming from blender, for example buttons to stop movement, etc.
                            # May move this logic out of the behavior tree...
                            # self.pollForBlenderInput(),

                            # Listen for audio input from people talking, etc.
                            # Again, this might not be the best place for this...
                            # self.listenForAudioInput()

                            self.test(),
                            self.updateVariables()
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(zoidSteinTree, blackboard=self.blackboard)

    def makeBasicZenoTree(self):
        ## Zeno's Body Paint subtree
        zenoBodyPaint = \
            owyl.selector(
                owyl.sequence(
                    self.isNotSameBrushStroke(),
                    self.isGreater(num1="animationOutputDur", num2=2),
                    self.setVariable(var="actionName", value="BrushStrokeGesture"),
                    owyl.selector(  # try the action sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(action="actionName")
                            # self.showAction(action=self.actionName, part=self.HEAD_NECK)  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say(utterance="I'm not feeling inspired today..."),
                            owyl.fail()
                        )
                    )
                ),
                owyl.sequence(
                    self.setVariable(var="actionName", value="Idle"),
                    owyl.selector(  # try the command sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(action="actionName")
                            # self.showAction(action=self.actionName, part=self.HEAD_NECK)  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say(utterance="Why can't I stand?"),
                            owyl.fail()
                        )
                    )
                )
            )

        ## body behavior subtree
        # zenoBodySubtree = \
        # Temp moved to the root tree

        # face & neck behavior subtree
        # zenoFaceSubtree = \
        # Temp moved to the root tree

        # Zeno's root tree
        zenoTree = \
            owyl.parallel(  # At the highest level, run several parallel behaviors
                # owyl.visit(zenoBodySubtree, blackboard=self.blackboard),
                # owyl.visit(zenoFaceSubtree, blackboard=self.blackboard),

                ##### zenoBodySubtree #####
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.selector(  # Select response to command or natural behavior
                            owyl.sequence(  # If the last audio or blender input is a command, then select a response
                                self.isCommand(key="audioInput"),
                                # self.setVariable(var=self.bodyOrFace, value=self.UPPER_BODY),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            # It's not a command, so start checking for natural behaviors
                            owyl.sequence(
                                self.isNoSalientTarget(),
                                self.isNoFaceTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),

                                # There's nothing to do, so let's paint!
                                # owyl.visit(zenoBodyPaint, blackboard=self.blackboard)
                            )
                            # TODO: the other natural actions, once we have a saliency target, etc.
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05
                ),

                ##### zenoFaceSubtree #####
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.selector(  # Select from one of several mutually exclusive face & neck behaviors
                            owyl.sequence(
                                self.isFaceTarget(),
                                self.isNoSalientTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),

                                self.faceGaze()
                                # owyl.visit(self.faceGaze, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isSalientTarget(),
                                self.isNoFaceTarget(),
                                self.isNoAudioInput(),
                                self.isNoRosInput(),
                                self.isNoEmotionInput(),

                                self.faceGaze()
                                # owyl.visit(self.faceGaze, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                owyl.selector(
                                    self.isAudioInput(),
                                    self.isRosInput()
                                ),
                                owyl.selector(
                                    owyl.sequence(
                                        self.isCommand(key="audioInput"),
                                        self.setVariable(var="commandInput", value="audioInput")
                                    ),
                                    owyl.sequence(
                                        self.isCommand(key="rosInput"),
                                        self.setVariable(var="commandInput", value="rosInput")
                                    ),
                                ),
                                # self.setVariable(var=self.bodyOrFace, value=self.HEAD_NECK),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isAudioInput(),
                                self.toDialSystem(key="audioInput")
                            )
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05
                ),

                ##### General tree #####
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            # Poll for input coming from blender, for example buttons to stop movement, etc.
                            # May move this logic out of the behavior tree...
                            # self.pollForBlenderInput(),

                            # Listen for audio input from people talking, etc.
                            # Again, this might not be the best place for this...
                            # self.listenForAudioInput(),

                            self.test(),
                            self.updateVariables()
                        )
                    ),
                    # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                    limit_period=0.05  # Testing
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(zenoTree, blackboard=self.blackboard)

    # def do_every(self, interval, worker_func, iterations=0):
    #     if iterations != 1:
    #         threading.Timer(interval, self.do_every, [interval, worker_func, 0 if iterations == 0 else iterations-1]).start()
    #     worker_func()

    def zenoDialCallback(self, data):
        print "(From ZenoDial) " + data.data
        self.itf_talk_pub.publish(data.data)  # For now
        # self.robot.say(data.data)
        self.blackboard["speechOutputAge"] = 0

    def audioInputCallback(self, data):
        self.blackboard["audioInputAge"] = 0
        # if not data.data == "":
        # self.audioInput = data.data
        self.blackboard["audioInput"] = data.data
        print "(From itf_listen) " + data.data

    def isSpeakingCallback(self, data):
        self.blackboard["speechActive"] = data.data

    def faceDetectCallback(self, data):
        # Loop through each pair of the input coordinates
        person_id = 0
        threshold = 100
        max_num_people = 4
        key_of_oldest = 0
        key_of_closest = 0
        min_velocity = -1

        # Indexes of person_detail:
        index_person = 0
        index_x = 1
        index_y = 2
        index_vel = 3
        index_age = 4
        index_dage = 5

        # Loop through each of the people from the input
        for person in range(len(data.positions)):
            x = data.positions[person].x
            y = data.positions[person].y
            print "Person " + str(person_id) + " (" + str(x) + ", " + str(y) + "), Length = " + str(len(self.blackboard["faceTarget"]))

            # Create a new person if there is none in the system
            # if len(self.blackboard["faceTarget"]) == person_id:
            if len(self.blackboard["faceTarget"]) == 0:
                self.blackboard["faceTarget"][person_id] = [Person(person_id), x, y, 0, 0, 0]
                print "New Person(" + str(person_id) + ")"
                person_id += 1
                continue

            velocity = math.sqrt(math.pow(x - self.blackboard["faceTarget"][person_id][index_x], 2) + math.pow(y - self.blackboard["faceTarget"][person_id][index_y], 2))

            # Same Person, update his coordinates
            if velocity < threshold:
                self.blackboard["faceTarget"][person_id] = [self.blackboard["faceTarget"][person_id][index_person], x, y, velocity, self.blackboard["faceTarget"][person_id][index_age], 0]
                # print "Same Person" + " (" + str(velocity) + ")"
            # # He is gone
            # else:
            #     self.blackboard["faceTarget"][person_id] = [self.blackboard["faceTarget"][person_id][index_person], x, y, velocity, self.blackboard["faceTarget"][person_id][index_age], self.blackboard["faceTarget"][person_id][index_dage] + 1]
            #     print "Missing Person(" + person_id + ")"

            person_id += 1

        # if no. of people in the system > no. of input coordinates
        # if len(self.blackboard["faceTarget"]) > person_id:
        #     for person_id in range(len(self.blackboard["faceTarget"])):
        #         self.blackboard["faceTarget"][person_id] = [self.blackboard["faceTarget"][person_id][index_person], self.blackboard["faceTarget"][person_id][index_x], self.blackboard["faceTarget"][person_id][index_y], self.blackboard["faceTarget"][person_id][index_vel], self.blackboard["faceTarget"][person_id][index_age], self.blackboard["faceTarget"][person_id][index_dage] + 1]
        #         print "Missing Person(" + person_id + ")!"

            # # Loop through each of the people in faceTarget and compare
            # for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            #     velocity = math.sqrt(math.pow(x - person_detail[index_x], 2) + math.pow(y - person_detail[index_y], 2))
            #
            #     # Keep track of the oldest
            #     if person_detail[index_age] > key_of_oldest:
            #         key_of_oldest = key
            #
            #     # Keep track of the closest
            #     if velocity < threshold and (velocity < min_velocity or min_velocity < 0):
            #         min_velocity = velocity
            #         key_of_closest = key
            #
            # # If velocity < threshold = Same person
            # if min_velocity >= 0:
            #     self.blackboard["faceTarget"][key_of_closest] = [self.blackboard["faceTarget"][key_of_closest][index_person], x, y, min_velocity, self.blackboard["faceTarget"][key_of_closest][index_age], 0]
            #     continue
            #
            # # If velocity > threshold = New person
            # if len(self.blackboard["faceTarget"]) < max_num_people:
            #     self.blackboard["faceTarget"][len(self.blackboard["faceTarget"]) + 1] = [Person(len(self.blackboard["faceTarget"]) + 1), x, y, 0, 0, 0]
            # # Replace the oldest with the new one
            # else:
            #     self.blackboard["faceTarget"][key_of_oldest] = [Person(key_of_oldest), x, y, 0, 0, 0]

    def saliencyCallback(self, data):
        self.blackboard["saliencyTarget"][0] = [Person(0), data.positions[0].x, data.positions[0].y, 0, 0]
        # TODO: Person should contain the coordinates!
        self.blackboard["robot"].gaze_and_wait(Person(0).head, speed=0.5)
        del self.blackboard["saliencyTarget"][0]

    def determineCurrentTarget(self):
        youngest_age = -1
        youngest_person = ""
        for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            if youngest_age < 0 or person_detail[4] < youngest_age:
                youngest_age = person_detail[4]  # person_detail[4] = age
                youngest_person = person_detail[0]  # person_detail[0] = person object
        self.blackboard["targetPos"] = youngest_person

    def removeFace(self):
        # print "Looking for faces to remove"
        disappear_threshold = 10
        people_to_del = []
        for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            if person_detail[5] > disappear_threshold:  # person_detail[5] = disappear_age
                people_to_del.append(key)
        for key in people_to_del:
            del self.blackboard["faceTarget"][key]

    @owyl.taskmethod
    def test(self, **kwargs):
        print "The tree is running..." + time.strftime("%Y%m%d%H%M%S")
        # self.zenodial_listen_pub.publish("Testing 123 please work")
        yield True

    @owyl.taskmethod
    def actionToPhrase(self, **kwargs):
        for (key, keywords) in self.blackboard["commandKeywords"].iteritems():
            for word in keywords:
                if self.blackboard["actionName"] == word:
                    self.blackboard["actionPhrase"] = key
        yield True

    @owyl.taskmethod
    def updateVariables(self, **kwargs):
        # Indexes of person_detail:
        index_person = 0
        index_x = 1
        index_y = 2
        index_vel = 3
        index_age = 4
        index_dage = 5

        self.blackboard["randomInput"] = random.random()
        # for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
        #     self.blackboard["faceTarget"][key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1, person_detail[index_dage]+1]
        # for (key, person_detail) in self.blackboard["saliencyTarget"].iteritems():
        #     self.blackboard["saliencyTarget"][key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1, person_detail[index_dage]+1]
        self.blackboard["audioInputAge"] = int(self.blackboard["audioInputAge"]) + 1
        self.blackboard["rosInputAge"] = int(self.blackboard["rosInputAge"]) + 1
        self.blackboard["emotionInputAge"] = int(self.blackboard["emotionInputAge"]) + 1
        self.blackboard["speechOutputAge"] = int(self.blackboard["speechOutputAge"]) + 1
        # self.blackboard["animationOutputAge"] = int(self.blackboard["animationOutputAge"]) + 1
        self.determineCurrentTarget()
        self.removeFace()
        yield True

    @owyl.taskmethod
    def isLess(self, **kwargs):
        if isinstance(kwargs["num1"], (int, long)):
            num1 = kwargs["num1"]
        else:
            num1 = self.blackboard[kwargs["num1"]]
        if isinstance(kwargs["num2"], (int, long)):
            num2 = kwargs["num2"]
        else:
            num2 = self.blackboard[kwargs["num2"]]
        if num1 < num2:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isGreater(self, **kwargs):
        if isinstance(kwargs["num1"], (int, long)):
            num1 = kwargs["num1"]
        else:
            num1 = self.blackboard[kwargs["num1"]]
        if isinstance(kwargs["num2"], (int, long)):
            num2 = kwargs["num2"]
        else:
            num2 = self.blackboard[kwargs["num2"]]
        if num1 > num2:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isSwitchingTarget(self):
        # TODO
        yield True

    @owyl.taskmethod
    def blink(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isSpeaking(self):
        yield self.blackboard["speechActive"]

    @owyl.taskmethod
    def say(self, **kwargs):
        self.blackboard["robot"].say(kwargs["utterance"])
        self.blackboard["speechOutputAge"] = 0
        yield True

    @owyl.taskmethod
    def sayStartAction(self, **kwargs):
        self.blackboard["robot"].say("I'll start to " + kwargs["actionName"] + "...")
        yield True

    @owyl.taskmethod
    def showAction(self, **kwargs):
        actionName = self.blackboard[kwargs["action"]]
        # part = kwargs["part"]

        if actionName == "Smile":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].show_expression(ZoidExpression.smile, 1.0)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].show_expression(ZenoExpression.smile, 1.0)

        elif actionName == "Frown Mouth":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].show_expression(ZoidExpression.frown_mouth, 1.0)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].show_expression(ZenoExpression.frown_mouth, 1.0)

        elif actionName == "Frown":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].show_expression(ZoidExpression.frown, 1.0)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].show_expression(ZenoExpression.frown, 1.0)

        elif actionName == "Open Mouth":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].show_expression(ZoidExpression.open_mouth, 1.0)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].show_expression(ZenoExpression.open_mouth, 1.0)

        elif actionName == "Wave":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.BConLeftArmWave)
                self.blackboard["robot"].gesture(ZoidGestureData.BConEightArmWave)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.BConLeftArmWave)
                self.blackboard["robot"].gesture(ZenoGestureData.BConEightArmWave)

        elif actionName == "Idle":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.Idle)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.Idle)

        elif actionName == "Look Up":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.LookUp)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.LookUp)

        elif actionName == "Look Down":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.LookDown)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.LookDown)

        elif actionName == "Point Up":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.PointUp)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.PointUp)

        elif actionName == "Point Down":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.PointDown)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.PointDown)

        elif actionName == "Walk Forward":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.WalkForward1)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.WalkForward1)

        elif actionName == "Walk Backward":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.WalkBackward1)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.WalkBackward1)

        elif actionName == "Turn Left":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.TurnLeft)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.TurnLeft)

        elif actionName == "Turn Right":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.TurnRight)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.TurnRight)

        elif actionName == "Stop":
            if self.blackboard["robotName"] == "Zoid":
                self.blackboard["robot"].gesture(ZoidGestureData.Stop)
            elif self.blackboard["robotName"] == "Zeno":
                self.blackboard["robot"].gesture(ZenoGestureData.Stop)

        # elif self.commandName == "Stop":
            # TODO: Stop and reset all the body movements

        yield True

    @owyl.taskmethod
    def faceTrack(self, **kwargs):
        pos = kwargs["pos"]
        eyeFree = kwargs["eyeFree"]
        neckFree = kwargs["neckFree"]
        rand = kwargs["rand"]
        yield True

    @owyl.taskmethod
    def faceGaze(self, **kwargs):
        self.blackboard["robot"].gaze_and_wait(self.blackboard["targetPos"].head, speed=0.5)
        yield True

    @owyl.taskmethod
    def isDefaultStance(self):
        # TODO
        yield True

    @owyl.taskmethod
    def resetToDefaultStance(self):
        self.blackboard["robot"].show_expression_and_wait(1, 0.0)
        self.blackboard["robot"].show_expression_and_wait(2, 0.0)
        self.blackboard["robot"].show_expression_and_wait(3, 0.0)
        self.blackboard["robot"].show_expression_and_wait(4, 0.0)
        # TODO: Reset other body parts
        yield True

    @owyl.taskmethod
    def toDialSystem(self, **kwargs):
        utterance = self.blackboard.get(kwargs["key"])

        print "Sending \"" + utterance + "\" to ZenoDial"
        self.zenodial_listen_pub.publish(utterance)
        self.blackboard["audioInput"] = ""
        yield True

    @owyl.taskmethod
    def isCommand(self, **kwargs):
        commandName = self.blackboard.get(kwargs["key"])

        found = False
        for (key, keywords) in self.blackboard["commandKeywords"].iteritems():
            for word in keywords:
                if commandName == word > -1:
                    found = True
                    self.blackboard["audioInput"] = ""
                    yield True
        if not found:
            yield False

    @owyl.taskmethod
    def isCommandPhrase(self, **kwargs):
        if blackboard[kwargs["commandName"]] == blackboard[kwargs["actionPhrase"]]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isVariable(self, **kwargs):
        yield self.blackboard[kwargs["var"]] == kwargs["value"]

    @owyl.taskmethod
    def setVariable(self, **kwargs):
        self.blackboard[kwargs["var"]] = self.blackboard[kwargs["value"]]
        yield True

    @owyl.taskmethod
    def isSalientTarget(self, **kwargs):
        if len(self.blackboard["saliencyTarget"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isFaceTarget(self, **kwargs):
        if len(self.blackboard["faceTarget"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoFaceTarget(self, **kwargs):
        if len(self.blackboard["faceTarget"]) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoSalientTarget(self, **kwargs):
        if len(self.blackboard["saliencyTarget"]) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isAudioInput(self, **kwargs):
        if not self.blackboard["audioInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoAudioInput(self, **kwargs):
        if self.blackboard["audioInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isRosInput(self, **kwargs):
        if not self.blackboard["rosInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoRosInput(self, **kwargs):
        if self.blackboard["rosInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoEmotionInput(self, **kwargs):
        if self.blackboard["emotionInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNotSameBrushStroke(self, **kwargs):
        # TODO
        yield True

    @owyl.taskmethod
    def dance(self, **kwargs):
        self.blackboard["robot"].gesture(ZoidGestureData.BConDance)
        yield True

class TreeLayer(ScrollableLayer):
    is_event_handler = True

    def __init__(self, tree_name):
        super(TreeLayer, self).__init__()
        self.tree_name = tree_name
        self.manager = ScrollingManager()
        self.manager.add(self)
        self.active = None
        self.tree = None

    def makeTree(self):
        self.tree = Tree(self.tree_name)

    def on_enter(self):
        super(TreeLayer, self).on_enter()
        self.makeTree()
        self.manager.set_focus(0, 0)


if __name__ == "__main__":
    '''The main entry point...'''
    rospy.loginfo("ITF Demo startup\n")
    world = World()

    # import sys
    # if len(sys.argv) == 2:
    #     tree_name = int(sys.argv[1])
    # else:
    #     tree_name = "default:"

    # tree_name = "BasicZenoTree"
    tree_name = "BasicZoidSteinTree"
    director.init(resizable=True, caption="Owyl Behavior Tree Demo :" + tree_name, width=1, height=1)
    s = Scene(TreeLayer(tree_name))
    director.run(s)