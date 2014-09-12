

__author__ = 'lake'
__revision__ = 0.1
__date__ = "$Date$"[7:-2]

import roslib
import rospy
import numpy
import math
import os
import subprocess
import re
import time
import numpy as np
import tf
import socket
import std_msgs
import threading

import sys
import random

### Memojito provides memoization (caching) services.
import memojito

### Pyglet provides graphics and resource management.
import pyglet

import owyl

from numpy import linalg

from math import radians, degrees, sin, cos, pi, atan2
pi_2 = pi*2.0
pi_1_2 = pi/2.0
pi_1_4 = pi/4.0
pi_3_4 = (pi*3)/4

### Optimized attribute getters for sprites..
from operator import attrgetter
getX = attrgetter('x')
getY = attrgetter('y')
getR = attrgetter('rotation')

pyglet.resource.path = [os.path.dirname(os.path.abspath(__file__)),]
pyglet.resource.reindex()

## Cocos provides scene direction and composition
from cocos.director import director
from cocos.scene import Scene
from cocos.actions import FadeIn
from cocos.layer import ScrollableLayer, ScrollingManager

## Rabbyt provides collision detection
from rabbyt.collisions import collide_single

## Owyl provides the wisdom
from owyl import blackboard

from steering import Steerable

from geometry_msgs.msg import Twist
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
        for case in switch(tree_name):
            if case("BasicZenoTree"):
                self.makeBasicZenoTree()
                break
            if case("BasicZoidSteinTree"):
                self.makeBasicZoidSteinTree()
                break
            if case():
                rospy.loginfo("Unrecognized Tree Name!\n")

        self.BehaviorNode = rospy.init_node("BehaviorNode")
        rospy.Subscriber("speech_active", Bool, self.speakingcallback)
        self.zenodial_listen_pub = rospy.Publisher("zenodial_listen", String, queue_size=1)
        self.itf_talk_pub = rospy.Publisher("itf_talk", String, queue_size=1)
        self.robot_movement_pub = rospy.Publisher("robot_movement", String, queue_size=1)
        self.commandKeywords = {
                'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me', 'abhor', 'a whore', 'a bore'],
                'slower': ['slow down', 'slower'],
                'faster': ['speed up', 'faster'],
                'forward': ['forward', 'ahead', 'straight', 'forwards'],
                'backward': ['back', 'backward', 'back up', 'backwards'],
                'rotate left': ['rotate left', 'rotates left', 'rotate lefts'],
                'rotate right': ['rotate right', 'rotates right', 'rotate rights'],
                'turn left': ['turn left', 'turn lefts', 'turns left'],
                'turn right': ['turn right', 'turn rights', 'turns right'],
                'quarter': ['quarter speed', 'quarters speed'],
                'half': ['half speed', 'halfs speed', 'halve speed'],
                'full': ['full speed', 'fulls speed'],
                'stop listen': ['pause listen', 'stop listen', 'pause listening', 'stop listening', 'top listen', 'top listening', 'topless in', 'end listen', 'endless in'],
                'start listen': ['continue listen', 'resume listen', 'start listen', 'continue listening', 'resume listening', 'start listening', 'dart listen', 'art listen', 'dart listening', 'art listening', 'begin listen', 'begin listening'],
                'start follow': ['continue follow', 'resume follow', 'start follow', 'continue following', 'resume following', 'start following', 'dart follow', 'art follow', 'dart following', 'art following', 'begin follow', 'begin following'],
                'stop follow': ['pause follow', 'stop follow', 'pause following', 'stop following', 'top follow', 'top following', 'end follow', 'end following'],
                'start speech': ['continue speech', 'resume speech', 'start speech', 'continue speaking', 'resume speaking', 'start speaking', 'art speech', 'dart speech', 'art speaking', 'dart speaking', 'begin speech', 'begin speaking'],
                'stop speech': ['pause speech', 'stop speech', 'pause speaking', 'stop speaking', 'top speech', 'top speaking', 'top speed', 'end speech', 'end speaking']}

        ### Inputs
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
        self.animationOutputAge = ""                # time since the last animation output from the robot
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
        self.actionName = ""
        self.bodyOrFace = ""
        self.targetPos = [[0.0, 0.0]]
        self.glanceOrSaccadeTargetPos = [[0.0, 0.0]]
        self.firstGreeting = False
        self.speechActive = False
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
                            # blink 50% more often when switching targets
                            self.isLess(num1=self.randomInput, num2=self.blinkChance*1.5),
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
        # Assumes bodyOrFace has been set
        # Will announce the command (if not already speaking)
        # before showing the associated animation
        self.executeBasicCommandSubtree = \
            owyl.sequence(
                self.isCommand(commandName=self.commandName),
                self.setVariable(var=self.actionName, value=self.commandName),
                owyl.selector(  # try the command sequence (subtree) or report failure
                    owyl.sequence(
                        owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                        self.showCommand(commandName=self.commandName, part=self.bodyOrFace),  # Finally play the command's animation
                    ),
                    owyl.sequence(
                        self.say(utterance="I'm sorry, Dave, I'm afraid I can't do that..."),
                        owyl.fail()
                    )
                )
            )

        ## Select a basic command to execute, once we know that we've been given a command.
        # Assumes bodyOrFace has been set, to distinguish body actions from face actions
        self.selectBasicCommandSubtree = \
            owyl.selector(  # Select from one of several mutually exclusive behaviors
                owyl.sequence(  # If we should be idling, then try to play the Idle animation...
                    self.setVariable(var=self.commandName, value="Idle"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we're commanded to anc can walk to target location, then play the walk animation until we reach the target
                    self.setVariable(var=self.commandName, value="WalkForward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="WalkBackward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="TurnLeft"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="TurnRight"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="PointUp"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(var=self.commandName, value="PointDown"),
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
                    self.setVariable(var=self.commandName, value="OpenMouth"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                )
            )

        ## Tracks the target face or salient point
        # Assumes targetPos has been set to face, body, or salient position
        self.faceGaze = \
            owyl.sequence(
                # TODO: Get clarification from Hanson and others on the conditions for tracking
                # owyl.selector(
                #   self.isFaceNearestAudioSource(self.faceTargetPos), # Do we have the source of the audio?
                #   self.isFaceMostSalient(self.faceTargetAge, self.saliencyTargetAge), # Do we know the degree/magnitude of saliency?
                #   self.isFaceCentered(self.faceTargetPos), # Can we find the centroid of all the faces?
                #   self.isLess(self.randomInput, self.blinkChance*2.0) # Should we really switch tracking targets this often?
                # ),

                # self.faceTrack(pos=targetPos, eyeFree=eyeFreedom, neckFree=neckFreedom, rand=-1)  # -1 here indicating that we'll track this point until told to stop
            )

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
                self.showAction(action="OpenMouth", part=self.HEAD_NECK)
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
                self.showAction(action="LookUp", part=self.HEAD_NECK),
                # self.showAction(action="Afraid", part="face")
                self.showAction(action="Frown", part=self.HEAD_NECK)
            )

        ## When people are very close, move head forward and down while playing the innoscent expression animation.
        self.shy = \
            owyl.sequence(
                self.showAction(action="LookDown", part=self.HEAD_NECK),
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

    def makeBasicZoidSteinTree(self):
        ## The scripted dance of ZoidStein, used when no faces or salient targets are detected.
        # Assumes body state has been reset to starting state
        zoidSteinBodyDance = \
            owyl.sequence(
                self.showCommand(commandName="WalkForward", part=self.LOWER_BODY),
                self.showCommand(commandName="WalkBackward", part=self.LOWER_BODY),
                self.showCommand(commandName="PointUp", part=self.UPPER_BODY),
                self.showCommand(commandName="PointDown", part=self.UPPER_BODY)
            )

        ## body behavior subtree
        # TODO: Attach subtrees properly
        zoidSteinBodySubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select response to command or natural behavior
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(commandName=self.commandName),
                            # self.setVariable(var=self.bodyOrFace, value="body"),
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
                            owyl.visit(zoidSteinBodyDance, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
                            #TODO: the other natural actions, once we have a saliency target, etc.
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
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(commandName=self.commandName),
                            self.setVariable(var=self.bodyOrFace, value="face"),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
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
                            self.isFaceTarget(),
                        )
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        ## ZoidStein's root tree
        zoidSteinTree = \
            owyl.parallel(  # At the highest level, run several parallel behaviors
                owyl.visit(zoidSteinBodySubtree, blackboard=self.blackboard),
                owyl.visit(zoidSteinFaceSubtree, blackboard=self.blackboard),
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            # Poll for input coming from blender, for example buttons to stop movement, etc.
                            # May move this logic out of the behavior tree...
                            self.pollForBlenderInput(),
                            # Listen for audio input from people talking, etc.
                            # Again, this might not be the best place for this...
                            self.listenForAudioInput()
                        )
                    ),
                    limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
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
                    self.setVariable(self.actionName, "BrushStrokeGesture"),
                    owyl.selector(  # try the action sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(self.actionName),  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say("I'm not feeling inspired today..."),
                            owyl.fail()
                        )
                    )
                ),
                owyl.sequence(
                    self.setVariable(self.actionName, "Idle"),
                    owyl.selector(  # try the command sequence (subtree) or report failure
                        owyl.sequence(
                            owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                            self.showAction(self.actionName),  # Finally play the action's animation
                        ),
                        owyl.sequence(
                            self.say("Why can't I stand?"),
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
                            self.isCommand(self.commandName),
                            self.setVariable(self.bodyOrFace, "body"),
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
                        ),
                        owyl.sequence(
                            # TODO: the other natural actions, once we have a saliency target, etc.
                        )
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        # face & neck behavior subtree
        zenoFaceSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select from one of several mutually exclusive face & neck behaviors
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(self.commandName),
                            self.setVariable(self.bodyOrFace, "face"),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
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
                            self.isFaceTarget()
                        )
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        # Zeno's root tree
        zenoTree = \
            owyl.parallel(  # At the highest level, run several parallel behaviors
                owyl.visit(zenoBodySubtree, blackboard=self.blackboard),
                owyl.visit(zenoFaceSubtree, blackboard=self.blackboard),
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            # Poll for input coming from blender, for example buttons to stop movement, etc.
                            # May move this logic out of the behavior tree...
                            self.pollForBlenderInput(),
                            # Listen for audio input from people talking, etc.
                            # Again, this might not be the best place for this...
                            self.listenForAudioInput()
                        )
                    ),
                    limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )

        return owyl.visit(zenoTree, blackboard=self.blackboard)

    def generateRandomInput(self):
        self.randomInput = random.random()

    def speakingcallback(self, data):
        self.speechActive = data.data

    def actionToPhrase(self, commandName):
        for (key, keywords) in self.commandKeywords.iteritems():
            for word in keywords:
                if commandName.find(word) > -1:
                    return key

    @owyl.taskmethod
    def toZenoDial(self, **kwargs):
        self.zenodial_listen_pub.publish(kwargs["utterance"])
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
        self.itf_talk_pub.publish(kwargs["utterance"])
        yield True

    @owyl.taskmethod
    def sayStartAction(self, **kwargs):
        self.itf_talk_pub.publish("I'll start " + kwargs["actionName"] + "...")
        yield True

    @owyl.taskmethod
    def showAction(self, **kwargs):
        self.actionName = kwargs["action"]
        self.bodyOrFace = kwargs["part"]
        self.robot_movement_pub.Publish(self.actionName + "-" + self.bodyOrFace)

    @owyl.taskmethod
    def faceTrack(self, **kwargs):
        pos = kwargs["pos"]
        eyeFree = kwargs["eyeFree"]
        neckFree = kwargs["neckFree"]
        rand = kwargs["rand"]
        # TODO

    @owyl.taskmethod
    def isDefaultStance(self):
        # TODO
        yield True

    @owyl.taskmethod
    def resetToDefaultStance(self):
        # TODO
        yield None

    @owyl.taskmethod
    def isCommand(self, **kwargs):
        self.commandName = kwargs["commandName"]
        for (key, keywords) in self.commandKeywords.iteritems():
            for word in keywords:
                if self.commandName.find(word) > -1:
                    yield True
        yield False

    @owyl.taskmethod
    def isVariable(self, **kwargs):
        yield kwargs['var'] == kwargs['value']

    @owyl.taskmethod
    def showCommand(self, **kwargs):  # Play the command animation
        commandName = kwargs["commandName"]
        part = kwargs["part"]
        message = Message()
        message.textPart = "commandName"
        showActionWithMessage(message)
        yield True

    @owyl.taskmethod
    def showWalkForward(self, **kwargs):
        # Below is for demonstration only
        message = Message()
        message.textPart = "WalkForward"
        message.posPart = walkTargetPos
        showActionWithMessage(message)
        yield True

    @owyl.taskmethod
    def setVariable(self, **kwargs):
        var = kwargs["var"]
        value = kwargs["value"]
        var = value
        yield True

    @owyl.taskmethod
    def showActionWithMessage(self, **kwargs):
        message = kwargs["message"]
        ##self.BehaviorNode.send()
        yield True

    @owyl.taskmethod
    def isNoSalientTarget(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isNoFaceTarget(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isNoAudioInput(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isNoRosInput(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isNoEmotionInput(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isFaceTarget(self):
        # TODO
        yield True

    def isSalientTarget(self):
        # TODO
        yield True

    @owyl.taskmethod
    def isNotSameBrushStroke(self):
        # TODO
        yield True

    @owyl.taskmethod
    def startMove(self, **kwargs):
        yield True

    @owyl.taskmethod
    def canEndMove(self, **kwargs):
        yield True

    @owyl.taskmethod
    def endMove(self, **kwargs):
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

        self.manager.set_focus(0,0)


if __name__ == "__main__":
    '''The main entry point...'''
    rospy.loginfo("ITF Demo startup\n")

    import sys
    if len(sys.argv) == 2:
        tree_name = int(sys.argv[1])
    else:
        tree_name = "default:"

    director.init(resizable=True, caption="Owyl Behavior Tree Demo :" + tree_name, width=1024, height=768)

    s = Scene(TreeLayer(tree_name))
    director.run(s)
