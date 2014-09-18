

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

# This class provides the functionality we want. You only need to look at
# this if you want to know how this works. It only needs to be defined
# once, no need to muck around with its internals.
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class Tree:
    def __init__(self, tree_name):
        # self.BehaviorNode = rospy.init_node("behavior_tree")
        rospy.Subscriber("itf_listen", String, self.audioInputCallback)
        rospy.Subscriber("speech_active", Bool, self.isSpeakingCallback)
        rospy.Subscriber("facedetect", targets, self.faceDetectCallback)
        rospy.Subscriber("nmpt_saliency_point", targets, self.saliencyCallback)
        self.zenodial_listen_pub = rospy.Publisher("zenodial_listen", String, queue_size=1)
        self.robot_movement_pub = rospy.Publisher("robot_movement", String, queue_size=1)
        self.commandKeywords = {
                # 'Stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me', 'abhor', 'a whore', 'a bore'],
                'Walk Forward': ['forward', 'ahead', 'straight', 'forwards'],
                'Walk Backward': ['back', 'backward', 'back up', 'backwards'],
                'Turn Left': ['turn left', 'turn lefts', 'turns left'],
                'Turn Right': ['turn right', 'turn rights', 'turns right']}

        ### Inputs
        self.faceTarget = {}                        # seq no: [Person obj, x, y, velocity, age]
        self.saliencyTarget = {}                    # seq no: [Person obj, x, y, velocity, age]
        self.saliencyTargetPos = [0.0, 0.0]         # position of the current saliency target
        self.saliencyTargetVel = 0.0                # average velocity of the current saliency target over the last second
        self.saliencyTargetAge = 0                  # time since the last significant change in the saliency target position
        self.faceTargetPos = [[0.0, 0.0]]           # position of the current face target
        self.faceTargetVel = 0.0                    # average velocity of the current face target over the last second
        self.faceTargetAge = 0                      # time since the last significant change in the face target position
        self.bodyTargetPos = [[0.0, 0.0]]           # position of the current body target
        self.bodyTargetVel = 0.0                    # average velocity of the current body target over the last second
        self.bodyTargetAge = 0                      # time since the last significant change in the body target position
        self.audioInput = ""                        # string output of speech-to-text algorithm, raw form
        self.audioInputAge = 0                      # time since the last significant parse of the audio input
        self.audioInputVol = 0                      # average volume or magnitude of the last audio input
        self.rosInput = ""                          # string representation of miscellaneous commands from other ros components, usually blender
        self.rosInputAge = 0                        # time since the last ros command
        self.emotionInput = ""                      # string output of the audio-emotion-analysis algorithm
        self.emotionInputAge = 0                    # time since the last significant chance in emotional state
        self.speechOutput = ""                      # string representation of the last speech output from the robot
        self.speechOutputAge = 0                    # time since the last speech output from the robot
        self.animationOutput = ""                   # string representation of the last animation output from the robot
        self.animationOutputAge = 0                 # time since the last animation output from the robot
        self.animationOutputDur = 0                 # for zeno body paint
        self.randomInput = 0                        # a random percentile for random behaviors

        ### Globals
        self.blinkChance = 0.011    # @ 60 fps a 1.1% chance to start a blink each frame should give us a nice frequency
        self.highBodyVel = 1        # Not sure what would be considered a high velocity for the body - use 1 for now
        self.eyeFreedom = 0
        self.neckFreedom = 0
        self.HEAD_NECK = "headneck"
        self.UPPER_BODY = "ubody"
        self.LOWER_BODY = "lbody"

        ### Locals
        self.commandName = ""
        self.commandInput = ""
        self.actionName = ""
        self.bodyOrFace = ""
        self.targetPos = ""
        self.glanceOrSaccadeTargetPos = {}
        self.firstGreeting = False
        self.speechActive = False
        self.robotName = ""
        self.blackboard = blackboard.Blackboard()

        ### Subtrees
        ## Blink Subtree. A small example of a tree to run in parallel with the other subtrees.
        # Assumes randomInput is recalculated each frame
        self.blinkSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(
                        owyl.sequence(
                            self.isSwitchingTarget(),
                            self.isLess(num1=self.randomInput, num2=self.blinkChance*1.5),  # blink 50% more often when switching targets
                            self.blink()
                        ),
                        owyl.sequence(
                            owyl.selector(
                                self.isGreater(num1=linalg.norm(self.bodyTargetVel), num2=1),
                                self.isSpeaking()
                            ),
                            self.isLess(num1=self.randomInput, num2=self.blinkChance*1.2),
                            self.blink()
                        ),
                        owyl.sequence(
                            self.isLess(num1=self.randomInput, num2=self.blinkChance),
                            self.blink()
                        )
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        ## Announce the action we're about to take and then reset the robot to a default stance.
        # Though we announce an action, this tree doesn't execute the action.
        # Assumes actionName has been set
        self.announceAndResetTree = \
            owyl.sequence(  # announce the action and then reset
                owyl.selector(  # If we're not speaking, then speak
                    self.isSpeaking(),
                    self.sayStartAction(commandName=self.actionToPhrase(self.actionName))
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
                self.isCommandPhrase(commandName=self.commandName, commandInput=self.commandInput),
                self.setVariable(var=self.actionName, value=self.commandName),
                owyl.selector(  # try the command sequence (subtree) or report failure
                    owyl.sequence(
                        owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                        self.showAction(commandName=self.actionName, part=self.bodyOrFace),  # Finally play the command's animation
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
                    self.setVariable(var=self.commandName, value="Idle"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Stop Speech"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we're commanded to anc can walk to target location, then play the walk animation until we reach the target
                    self.setVariable(var=self.commandName, value="Walk Forward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Walk Backward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Turn Left"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Turn Right"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Point Up"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Point Down"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Look Up"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Look Down"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Wave"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we should show an emotion, then select the right one and show it.
                    self.setVariable(var=self.commandName, value="Smile"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Frown"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Frown Mouth"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="Open Mouth"),
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
                    self.isGreater(num1=self.audioInputVol, num2=1),  # or whatever counts for a high volume
                    self.isGreater(num1=linalg.norm(self.faceTargetVel), num2=1),
                    self.isGreater(num1=linalg.norm(self.bodyTargetVel), num2=1),
                    self.isGreater(num1=linalg.norm(self.saliencyTargetVel), num2=1)
                ),
                self.showAction(action="Open Mouth", part=self.HEAD_NECK)
                # self.showAction(action="Surprised", part="face")
            )

        ## Random eye movements which are much faster and less pronounced than glances.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        # Assumes eyeFreedom and neckFreedom have been set to appropriate degrees of freedom
        self.saccade = \
            owyl.selector(
                owyl.sequence(
                    self.isGreater(num1=self.randomInput, num2=0.5),
                    self.faceTrack(pos=self.glanceOrSaccadeTargetPos, eyeFree=self.randomInput*0.25*self.eyeFreedom, neckFree=self.randomInput*0.10*self.neckFreedom, rand=self.randomInput)
                ),
                owyl.sequence(
                    self.faceTrack(pos=self.glanceOrSaccadeTargetPos, eyeFree=self.randomInput*0.75*self.eyeFreedom, neckFree=self.randomInput*0.30*self.neckFreedom, rand=self.randomInput)
                )
            )

        ## Random eye movements which signal recognition of targets.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        self.glance = \
            owyl.selector(
                owyl.sequence(
                    owyl.selector(
                        self.isLess(num1=self.faceTargetAge, num2=1),
                        self.isLess(num1=self.randomInput, num2=0.0025)
                    ),
                    self.faceTrack(pos=self.glanceOrSaccadeTargetPos, eyeFree=self.eyeFreedom, neckFree=self.neckFreedom, rand=self.randomInput*2.5)
                ),
                owyl.sequence(
                    owyl.selector(
                        self.isLess(num1=self.saliencyTargetAge, num2=1),
                        self.isLess(num1=self.randomInput, num2=0.0025)
                    ),
                    self.faceTrack(pos=self.glanceOrSaccadeTargetPos, eyeFree=self.eyeFreedom, neckFree=self.neckFreedom, rand=self.randomInput*2.5)
                )
            )

        ## After tracking at a new target face, ZoidStein will execute a scripted greeting.
        # Be careful not to play this more than once in the same encounter.
        self.greeting = \
            owyl.sequence(
                self.isVariable(var=self.firstGreeting, value=False),
                self.setVariable(var=self.firstGreeting, value=True),
                self.say(utterance="Hello!"),
                self.showAction(action="Wave", part=self.UPPER_BODY),
                self.showAction(action="Smile", part=self.HEAD_NECK)
            )

        ## When people are too close, move head back and up while playing the afraid expression animation.
        self.awkward = \
            owyl.sequence(
                self.showAction(action="Look Up", part=self.HEAD_NECK),
                # self.showAction(action="Afraid", part="face")
                self.showAction(action="Frown", part=self.HEAD_NECK)
            )

        ## When people are very close, move head forward and down while playing the innoscent expression animation.
        self.shy = \
            owyl.sequence(
                self.showAction(action="Look Down", part=self.HEAD_NECK),
                # self.showAction(action="Innocent", part=self.HEAD_NECK)
                self.showAction(action="Frown", part=self.HEAD_NECK)
            )

        ## In general, ZoidStein's expressions should mimic the emotional input of its targets.
        self.mimic = \
            owyl.selector(
                owyl.sequence(
                    self.isVariable(var=self.emotionInput, value="Happy"),
                    self.showAction(action="Happy", part=self.UPPER_BODY),
                    self.showAction(action="Happy", part=self.HEAD_NECK)
                )
                # TODO: Do we have to mimic all the emotionInput or just happy?
            )

        """
        Creating the tree
        """
        for case in switch(tree_name):
            if case("BasicZenoTree"):
                # self.robot = Zeno()
                self.robotName = "Zeno"
                self.tree = self.makeBasicZenoTree()
                self.do_every(0.01, self.tree.next)
                break
            if case("BasicZoidSteinTree"):
                self.robot = Zoidstein()
                self.robotName = "Zoid"
                self.tree = self.makeBasicZoidSteinTree()
                self.do_every(0.01, self.tree.next)
                break
            if case():
                rospy.loginfo("Unrecognized Tree Name!\n")

    def makeBasicZoidSteinTree(self):
        ## The scripted dance of ZoidStein, used when no faces or salient targets are detected.
        # Assumes body state has been reset to starting state
        # zoidSteinBodyDance = \
        #     owyl.sequence(
        #         self.showCommand(commandName="WalkForward", part=self.LOWER_BODY),
        #         self.showCommand(commandName="WalkBackward", part=self.LOWER_BODY),
        #         self.showAction(commandName="PointUp", part=self.UPPER_BODY),
        #         self.showAction(commandName="PointDown", part=self.UPPER_BODY)
        #     )

        ## body behavior subtree
        # TODO: Attach subtrees properly
        zoidSteinBodySubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select response to command or natural behavior
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(commandName=self.audioInput),
                            self.setVariable(var=self.bodyOrFace, value="body"),
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
                            self.dance()
                            # owyl.visit(zoidSteinBodyDance, blackboard=self.blackboard)
                        )
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        ## face & neck behavior subtree
        # TODO: Attach subtrees properly
        zoidSteinFaceSubtree = \
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
                                    self.isCommand(commandName=self.audioInput),
                                    self.setVariable(var=self.commandInput, value=self.audioInput)
                                ),
                                owyl.sequence(
                                    self.isCommand(commandName=self.rosInput),
                                    self.setVariable(var=self.commandInput, value=self.rosInput)
                                ),
                            ),
                            self.setVariable(var=self.bodyOrFace, value=self.HEAD_NECK),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
                            self.isAudioInput(),
                            self.toDialSystem()
                        )
                    )
                ),
                # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                limit_period=0.05
            )

        ## ZoidStein's root tree
        zoidSteinTree = \
            owyl.parallel(  # At the highest level, run several parallel behaviors
                # owyl.visit(zoidSteinBodySubtree, blackboard=self.blackboard),
                # owyl.visit(zoidSteinFaceSubtree, blackboard=self.blackboard),

                ##### zoidSteinBodySubtree #####
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.selector(  # Select response to command or natural behavior
                            owyl.sequence(  # If the last audio or blender input is a command, then select a response
                                self.isCommand(commandName=self.audioInput),
                                self.setVariable(var=self.bodyOrFace, value=self.UPPER_BODY),
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
                    limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
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
                                        self.isCommand(commandName=self.audioInput),
                                        self.setVariable(var=self.commandInput, value=self.audioInput)
                                    ),
                                    owyl.sequence(
                                        self.isCommand(commandName=self.rosInput),
                                        self.setVariable(var=self.commandInput, value=self.rosInput)
                                    ),
                                ),
                                self.setVariable(var=self.bodyOrFace, value=self.HEAD_NECK),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isAudioInput(),
                                self.toDialSystem()
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
                    self.isGreater(num1=self.animationOutputDur, num2=2),
                    self.setVariable(var=self.actionName, value="BrushStrokeGesture"),
                    owyl.selector(  # try the action sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(action=self.actionName, part=self.HEAD_NECK)  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say(utterance="I'm not feeling inspired today..."),
                            owyl.fail()
                        )
                    )
                ),
                owyl.sequence(
                    self.setVariable(var=self.actionName, value="Idle"),
                    owyl.selector(  # try the command sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(action=self.actionName, part=self.HEAD_NECK)  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say(utterance="Why can't I stand?"),
                            owyl.fail()
                        )
                    )
                )
            )

        ## body behavior subtree
        zenoBodySubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select response to command or natural behavior
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(commandName=self.audioInput),
                            self.setVariable(var=self.bodyOrFace, value=self.UPPER_BODY),
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
                            owyl.visit(zenoBodyPaint, blackboard=self.blackboard)
                        )
                        # TODO: the other natural actions, once we have a saliency target, etc.
                    )
                ),
                # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                limit_period=0.05
            )

        # face & neck behavior subtree
        zenoFaceSubtree = \
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
                                    self.isCommand(commandName=self.audioInput),
                                    self.setVariable(var=self.commandInput, value=self.audioInput)
                                ),
                                owyl.sequence(
                                    self.isCommand(commandName=self.rosInput),
                                    self.setVariable(var=self.commandInput, value=self.rosInput)
                                ),
                            ),
                            self.setVariable(var=self.bodyOrFace, value=self.HEAD_NECK),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
                            self.isAudioInput(),
                            self.toDialSystem()
                        )
                    )
                ),
                # limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                limit_period=0.05
            )

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
                                self.isCommand(commandName=self.audioInput),
                                self.setVariable(var=self.bodyOrFace, value=self.UPPER_BODY),
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
                                        self.isCommand(commandName=self.audioInput),
                                        self.setVariable(var=self.commandInput, value=self.audioInput)
                                    ),
                                    owyl.sequence(
                                        self.isCommand(commandName=self.rosInput),
                                        self.setVariable(var=self.commandInput, value=self.rosInput)
                                    ),
                                ),
                                self.setVariable(var=self.bodyOrFace, value=self.HEAD_NECK),
                                owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                            ),
                            owyl.sequence(
                                self.isAudioInput(),
                                self.toDialSystem()
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

    def do_every(self, interval, worker_func, iterations=0):
        if iterations != 1:
            threading.Timer(interval, self.do_every, [interval, worker_func, 0 if iterations == 0 else iterations-1]).start()
        worker_func()

    def actionToPhrase(self, commandName):
        for (key, keywords) in self.commandKeywords.iteritems():
            for word in keywords:
                if commandName.find(word) > -1:
                    return key

    def audioInputCallback(self, data):
        self.audioInputAge = 0
        self.audioInput = data.data

    def isSpeakingCallback(self, data):
        self.speechActive = data.data

    def faceDetectCallback(self, data):
        # Loop through each pair of the input coordinates
        for person in range(len(data.positions)):
            x = data.positions[person].x
            y = data.positions[person].y
            threshold = 100
            max_num_people = 4
            key_of_oldest = 0

            # Indexes of person_detail:
            index_person = 0
            index_x = 1
            index_y = 2
            index_vel = 3
            index_age = 4

            # Create a new person if there is none in the system
            if len(self.faceTarget) == 0:
                self.faceTarget[0] = [Person(0), x, y, 0, 0]
                return
            # Loop through each of the people in faceTarget and compare
            for (key, person_detail) in self.faceTarget.iteritems():
                velocity = math.sqrt(math.pow(x - person_detail[index_x], 2) + math.pow(y - person_detail[index_y], 2))
                # Find who is the oldest
                if person_detail[index_age] > key_of_oldest:
                    key_of_oldest = key
                # If velocity < threshold = Same person
                if velocity < threshold:
                    self.faceTarget[key] = [person_detail[index_person], x, y, velocity, person_detail[index_age]]
                    return
            # If velocity > threshold = New person
            if len(self.faceTarget) < max_num_people:
                self.faceTarget[len(self.faceTarget) + 1] = [Person(len(self.faceTarget) + 1), x, y, 0, 0]
            # Replace the oldest with the new one
            else:
                self.faceTarget[key_of_oldest] = [Person(key_of_oldest), x, y, 0, 0]

    def saliencyCallback(self, data):
        self.saliencyTarget[0] = [Person(0), data.positions[0].x, data.positions[0].y, 0, 0]

    @owyl.taskmethod
    def test(self, **kwargs):
        print "The tree is running..." + time.strftime("%Y%m%d%H%M%S")
        # self.zenodial_listen_pub.publish("Testing 123 please work")
        yield True

    @owyl.taskmethod
    def updateVariables(self, **kwargs):
        # Indexes of person_detail:
        index_person = 0
        index_x = 1
        index_y = 2
        index_vel = 3
        index_age = 4

        self.randomInput = random.random()
        for (key, person_detail) in self.faceTarget.iteritems():
            self.faceTarget[key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1]
        for (key, person_detail) in self.saliencyTarget.iteritems():
            self.saliencyTarget[key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1]
        self.audioInputAge += 1
        self.rosInputAge += 1
        self.emotionInputAge += 1
        self.speechOutputAge += 1
        self.animationOutputAge += 1
        yield True

    @owyl.taskmethod
    def isLess(self, **kwargs):
        if kwargs["num1"] < kwargs["num2"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isGreater(self, **kwargs):
        if kwargs["num1"] > kwargs["num2"]:
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
        yield self.speechActive

    @owyl.taskmethod
    def say(self, **kwargs):
        self.robot.say(kwargs["utterance"])
        yield True

    @owyl.taskmethod
    def sayStartAction(self, **kwargs):
        self.robot.say("I'll start " + kwargs["actionName"] + "...")
        yield True

    @owyl.taskmethod
    def showAction(self, **kwargs):
        actionName = kwargs["action"]
        part = kwargs["part"]

        if actionName == "Smile":
            if self.robotName == "Zoid":
                self.robot.show_expression(ZoidExpression.smile, 1.0)
            elif self.robotName == "Zeno":
                self.robot.show_expression(ZenoExpression.smile, 1.0)

        elif actionName == "Frown Mouth":
            if self.robotName == "Zoid":
                self.robot.show_expression(ZoidExpression.frown_mouth, 1.0)
            elif self.robotName == "Zeno":
                self.robot.show_expression(ZenoExpression.frown_mouth, 1.0)

        elif actionName == "Frown":
            if self.robotName == "Zoid":
                self.robot.show_expression(ZoidExpression.frown, 1.0)
            elif self.robotName == "Zeno":
                self.robot.show_expression(ZenoExpression.frown, 1.0)

        elif actionName == "Open Mouth":
            if self.robotName == "Zoid":
                self.robot.show_expression(ZoidExpression.open_mouth, 1.0)
            elif self.robotName == "Zeno":
                self.robot.show_expression(ZenoExpression.open_mouth, 1.0)

        elif actionName == "Wave":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.BConLeftArmWave)
                self.robot.gesture(ZoidGestureData.BConEightArmWave)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.BConLeftArmWave)
                self.robot.gesture(ZenoGestureData.BConEightArmWave)

        elif actionName == "Idle":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.Idle)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.Idle)

        elif actionName == "Look Up":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.LookUp)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.LookUp)

        elif actionName == "Look Down":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.LookDown)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.LookDown)

        elif actionName == "Point Up":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.PointUp)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.PointUp)

        elif actionName == "Point Down":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.PointDown)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.PointDown)

        elif actionName == "Walk Forward":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.WalkForward1)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.WalkForward1)

        elif actionName == "Walk Backward":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.WalkBackward1)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.WalkBackward1)

        elif actionName == "Turn Left":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.TurnLeft)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.TurnLeft)

        elif actionName == "Turn Right":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.TurnRight)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.TurnRight)

        elif actionName == "Stop":
            if self.robotName == "Zoid":
                self.robot.gesture(ZoidGestureData.Stop)
            elif self.robotName == "Zeno":
                self.robot.gesture(ZenoGestureData.Stop)

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
        self.robot.gaze_and_wait(self.targetPos.head, speed=0.5)
        yield True

    @owyl.taskmethod
    def isDefaultStance(self):
        # TODO
        yield True

    @owyl.taskmethod
    def resetToDefaultStance(self):
        self.robot.show_expression_and_wait(1, 0.0)
        self.robot.show_expression_and_wait(2, 0.0)
        self.robot.show_expression_and_wait(3, 0.0)
        self.robot.show_expression_and_wait(4, 0.0)
        # TODO: Reset other body parts
        yield True

    @owyl.taskmethod
    def toDialSystem(self, **kwargs):
        self.zenodial_listen_pub.publish(kwargs["utterance"])
        self.audioInput = ""

    @owyl.taskmethod
    def isCommand(self, **kwargs):
        commandName = kwargs["commandName"]
        found = False
        for (key, keywords) in self.commandKeywords.iteritems():
            for word in keywords:
                if commandName.find(word) > -1:
                    found = True
                    self.audioInput = ""
                    yield True
        if not found:
            yield False

    @owyl.taskmethod
    def isCommandPhrase(self, **kwargs):
        commandName = kwargs["commandName"]
        commandInput = kwargs["commandInput"]
        if commandName == self.actionToPhrase(commandInput):
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isVariable(self, **kwargs):
        yield kwargs['var'] == kwargs['value']

    @owyl.taskmethod
    def setVariable(self, **kwargs):
        kwargs["var"] = kwargs["value"]
        yield True

    @owyl.taskmethod
    def isSalientTarget(self, **kwargs):
        if len(self.saliencyTarget) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isFaceTarget(self, **kwargs):
        if len(self.faceTarget) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoFaceTarget(self, **kwargs):
        if len(self.faceTarget) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoSalientTarget(self, **kwargs):
        if len(self.saliencyTarget) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isAudioInput(self, **kwargs):
        if not self.audioInput == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoAudioInput(self, **kwargs):
        if self.audioInput == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isRosInput(self, **kwargs):
        if not self.rosInput == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoRosInput(self, **kwargs):
        if self.rosInput == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoEmotionInput(self, **kwargs):
        if self.emotionInput == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNotSameBrushStroke(self, **kwargs):
        # TODO
        yield True

    @owyl.taskmethod
    def dance(self, **kwargs):
        self.robot.gesture(ZoidGestureData.BConDance)
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