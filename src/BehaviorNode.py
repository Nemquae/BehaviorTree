

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
from std_msgs.msg import String
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

		### Inputs
        self.saliencyTargetPos = array([0, 0, 0])   # position of the current saliency target
        self.saliencyTargetVel = array([0, 0, 0])   # average velocity of the current saliency target over the last second
        self.saliencyTargetAge = 0                  # time since the last significant change in the saliency target position
        self.faceTargetPos = array([0, 0, 0])       # position of the current face target
        self.faceTargetVel = array([0, 0, 0])       # average velocity of the current face target over the last second
        self.faceTargetAge = 0                      # time since the last significant change in the face target position
        self.bodyTargetPos = array([0, 0, 0])       # position of the current body target
        self.bodyTargetVel = array([0, 0, 0])       # average velocity of the current body target over the last second
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
        self.randomInput = 0                        # a random percentile for random behaviors

        ### Globals
        self.blinkChance = 0.011    # @ 60 fps a 1.1% chance to start a blink each frame should give us a nice frequency
        self.highBodyVel = 1        # Not sure what would be considered a high velocity for the body - use 1 for now
        self.eyeFreedom = 0
        self.neckFreedom = 0

        ### Locals
        self.commandName = ""
        self.actionName = ""
        self.bodyOrFace = "body"
        self.targetPos = array([0, 0, 0])
        self.glanceOrSaccadeTargetPos = array([0, 0, 0])
        self.firstGreeting = False

		
        self.blackboard = blackboard.Blackboard()

        ### Subtrees

        ## Blink Subtree.  A small example of a tree to run in parallel with the other subtrees.
        # Assumes randomInput is recalculated each frame
        self.blinkSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(
                        owyl.sequence(
                            self.isSwitchingTarget(),
                            self.isLess(self.randomInput, self.blinkChance*1.5), # blink 50% more often when switching targets
                            self.blink()
                        ),
                        owyl.sequence(
                            owyl.selector(
                                self.isGreater(linalg.norm(self.bodyTargetVel), 1),
                                self.isSpeaking()
                            ),
                            self.isLess(self.randomInput, self.blinkChance*1.2),
                            self.blink()
                        ),
                        owly.sequence(
                            self.isLess(self.randomInput, self.blinkChance),
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
            owyl.sequence( # announce the action and then reset
                owyl.selector(  # If we're not speaking, then speak
                    self.isSpeaking(),
                    self.say("I'll start " + self.actionToPhrase(self.actionName) + "...")
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
                self.isCommand(self.commandName),
                self.setVariable(self.actionName, self.commandName),
                owyl.selector(  # try the command sequence (subtree) or report failure
                    owyl.sequence(
                        owyl.visit(self.announceAndResetTree, blackboard=self.blackboard),
                        self.showCommand(self.commandName, self.bodyOrFace),  # Finally play the command's animation
                    ),
                    owyl.sequence(
                         self.say("I'm sorry, Dave, I'm afraid I can't do that..."),
                         owyl.fail()
                    )
                )
            )


        ## Select a basic command to execute, once we know that we've been given a command.
		# Assumes bodyOrFace has been set, to distinguish body actions from face actions
        self.selectBasicCommandSubtree = \
            owyl.selector(  # Select from one of several mutually exclusive behaviors
                owyl.sequence(  # If we should be idling, then try to play the Idle animation...
                    self.setVariable(self.commandName, "Idle"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we're commanded to anc can walk to target location, then play the walk animation until we reach the target
                    self.setVariable(self.commandName, "WalkForward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "PointUp"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "WalkBackward"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "PointDown"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(  # If we should show an emotion, then select the right one and show it.
                    self.setVariable(self.commandName, "Happy"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Sad"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Surprised"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Evil"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Afraid"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Innocent"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Violent"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                ),
                owyl.sequence(
                    self.setVariable(self.commandName, "Eureka"),
                    owyl.visit(self.executeBasicCommandSubtree, blackboard=self.blackboard)
                )
                # TODO: Add more actions once we know the commands.
            )

        ## Tracks the target face or salient point
        # Assumes targetPos has been set to face, body, or salient position
        self.faceGaze = \
            owyl.sequence(
                # owyl.selector(
                #   self.isFaceNearestAudioSource(self.faceTargetPos), # Do we have the source of the audio?
                #   self.isFaceMostSalient(self.faceTargetAge, self.saliencyTargetAge), # Do we know the degree/magnitude of saliency?
                #   self.isFaceCentered(self.faceTargetPos), # Can we find the centroid of all the faces?
                #   self.isLess(self.randomInput, self.blinkChance*2.0) # Should we really switch tracking targets this often?
                # ),
                # TODO: Get clarification from Hanson and others on the conditions for tracking
                self.faceTrack(targetPos, eyeFreedom, neckFreedom, -1) # -1 here indicating that we'll track this point until told to stop
            )

        ## Displays the surprised emotional expression under certain conditions
        # Assumes targetPos has been set to face, body, or salient position
        self.startle = \
            owyl.sequence(
                owyl.selector(
                    self.isGreater(self.audioInputVol, 1), # or whatever counts for a high volume
                    self.isGreater(linalg.norm(self.faceTargetVel), 1),
                    self.isGreater(linalg.norm(self.bodyTargetVel), 1),
                    self.isGreater(linalg.norm(self.saliencyTargetVel), 1)
                ),
                self.showAction("Surprised", "face")
            )

        ## Random eye movements which are much faster and less pronounced than glances.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        # Assumes eyeFreedom and neckFreedom have been set to appropriate degrees of freedom
        self.saccade = \
            owyl.selector(
                owyl.sequence(
                    self.isGreater(self.randomInput, 0.5),
                    self.faceTrack(self.glanceOrSaccadeTargetPos, self.randomInput*0.25*self.eyeFreedom, self.randomInput*0.10*self.neckFreedom, self.randomInput)
                ),
                owyl.sequence(
                    self.faceTrack(self.glanceOrSaccadeTargetPos, self.randomInput*0.75*self.eyeFreedom, self.randomInput*0.30*self.neckFreedom, self.randomInput)
                )
            )


        ## Random eye movements which signal recognition of targets.
        # Assumes targetPos has been set to face or salient position
        # Assumes glanceOrSaccadeTargetPos has been set to face's body or salient point nearby
        self.glance = \
            owyl.selector(
                owyl.sequence(
                    owyl.selector(
                        self.isLess(self.faceTargetAge, 1),
                        self.isLess(self.randomInput, 0.0025)
                    ),
                    self.faceTrack(self.glanceOrSaccadeTargetPos, self.eyeFreedom, self.neckFreedom, self.randomInput*2.5)
                ),
                owyl.sequence(
                    owyl.selector(
                        self.isLess(self.saliencyTargetAge, 1),
                        self.isLess(self.randomInput, 0.0025)
                    ),
                    self.faceTrack(self.glanceOrSaccadeTargetPos, self.eyeFreedom, self.neckFreedom, self.randomInput*2.5)
                )
            )

        ## After tracking at a new target face, ZoidStein will execute a scripted greeting.
        # Be careful not to play this more than once in the same encounter.
        self.greeting = \
            owyl.sequence(
                self.isVariable(self.firstGreeting, False),
                self.setVariable(self.firstGreeting, True),
                self.say("Hello!"),
                self.showAction("Wave", "body"),
                self.showAction("Smile", "face")
            )

        ## When people are too close, move head back and up while playing the afraid expression animation.
        self.awkward = \
            owyl.sequence(
                self.showAction("LookUp", "face"),
                self.showAction("Afraid", "face")
            )

        ## When people are very close, move head forward and down while playing the innoscent expression animation.
        self.shy = \
            owyl.sequence(
                self.showAction("LookDown", "face"),
                self.showAction("Innocent", "face")
            )

        ## In general, ZoidStein's expressions should mimic the emotional input of its targets.
        self.mimic = \
            owyl.selector(
                owyl.sequence(
                    self.isVaraible(self.emotionInput, "Happy"),
                    self.showAction("Happy", "body"),
                    self.showAction("Happy", "face")
                )
            )


    def makeBasicZoidSteinTree(self):

        ## The scripted dance of ZoidStein, used when no faces or salient targets are detected.
        # Assumes body state has been reset to starting state
        self.zoidSteinBodyDance = \
            owyl.sequence(
                self.showCommand("WalkForward", "body"),
                self.showCommand("PointUp", "body"),
                self.showCommand("PointDown", "body"),
                self.showCommand("WalkBackward", "body")
            )

        ## body behavior subtree
        # TODO: Attach subtrees properly
        zoidSteinBodySubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select response to command or natural behavior
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(),
                            self.setVariable(self.bodyOrFace, "body"),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        # It's not a command, so start checking for natural behaviors
                        owyl.sequence(
                            self.IsNoSalientTarget(),
                            self.IsNoFaceTarget(),
                            self.IsNoAudioInput(),
                            self.IsNoRosInput(),
                            self.IsNoEmotionInput(),
                            # There's nothing to do, so let's dance!
                            owyl.visit(zoidSteinBodyDance, blackboard=self.blackboard)
                        ),

                        owyl.sequence(

                        ),
                        # the other natural actions, once we have a saliency target, etc.
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        ## face behavior subtree
        # TODO: Attach subtrees properly
        zoidSteinFaceSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select from one of several mutually exclusive face behaviors
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(),
                            self.setVariable(self.bodyOrFace, "face"),
                            owyl.visit(self.selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
							self.IsSalientTarget(),
							self.IsNoFaceTarget(),
							self.IsNoAudioInput(),
							self.IsNoRosInput(),
							self.IsNoEmotionInput(),
							owyl.visit(zoidSteinFaceGaze, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
							self.IsFaceTarget(),
                        )

                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

		## Zeno's root tree
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
					self.IsNotSameBrushStroke(),
					self.IsGreater(self.animationOutputDur, 2),
					self.setVariable(self.actionName, "BrushStrokeGesture"),
					owyl.selector(  # try the action sequence (subtree) or report failure
						owyl.sequence(
							owyl.visit(announceAndResetTree, blackboard=self.blackboard),
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
							owyl.visit(announceAndResetTree, blackboard=self.blackboard),
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
                            self.isCommand(),
                            self.setVariable(self.bodyOrFace, "body"),
                            owyl.visit(selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        # It's not a command, so start checking for natural behaviors
                        owyl.sequence(
                            self.IsNoSalientTarget(),
                            self.IsNoFaceTarget(),
                            self.IsNoAudioInput(),
                            self.IsNoRosInput(),
                            self.IsNoEmotionInput(),
                            # There's nothing to do, so let's paint!
                            owyl.visit(zenoBodyPaint, blackboard=self.blackboard)
                        ),
                        
                        owyl.sequence(
							
                        ),
                        # the other natural actions, once we have a saliency target, etc.
                    )
                ),
                limit_period=0.4  # Yield to the other behaviors after every 400 milliseconds of processing
            )

        # face behavior subtree
        zenoFaceSubtree = \
            owyl.limit(
                owyl.repeatAlways(
                    owyl.selector(  # Select from one of several mutually exclusive face behaviors
                        owyl.sequence(  # If the last audio or blender input is a command, then select a response
                            self.isCommand(),
                            self.setVariable(self.bodyOrFace, "face"),
                            owyl.visit(selectBasicCommandSubtree, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
							self.IsSalientTarget(),
							self.IsNoFaceTarget(),
							self.IsNoAudioInput(),
							self.IsNoRosInput(),
							self.IsNoEmotionInput(),
							owyl.visit(zenoFaceGaze, blackboard=self.blackboard)
                        ),
                        owyl.sequence(
							self.IsFaceTarget(),
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

    @owyl.taskmethod
    def showCommand(self, commandName, **kwargs):
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
	def setVariable(self, var, value, **kwargs):
		var = value
		yield True

    @owyl.taskmethod
    def showActionWithMessage(self, message, **kwargs):
        ##self.BehaviorNode.send()
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


if __name__=="__main__":
    '''The main entry point...'''
    rospy.loginfo("ITF Demo startup\n")

    # rospy.init_node('itf_demo')

    import sys
    if len(sys.argv) == 2:
        tree_name = int(sys.argv[1])
    else:
        tree_name = "default:"

    director.init(resizable=True, caption="Owyl Behavior Tree Demo :" + tree_name, width=1024, height=768)

    s = Scene(TreeLayer(tree_name))
    director.run(s)
