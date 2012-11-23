dojo.provide("museum.common.RobotPose");

dojo.declare("museum.common.RobotPose", null, {
	
	constructor: function() {
		this.head = {};
		this.torso = {};
		this.left = {};
		this.right = {};
	},
	
	setHead: function(pan, tilt) {
		this.head = [ pan, tilt ];
	},
	
	setTorso: function(lift) {
		this.torso = [ lift ];
	},
	
	setLeft: function(pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper) {
		this.left = [ pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper ];
	},
	
	setRight: function(pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper) {
		this.right = [ pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper ];
	},
	
	toDomNode: function() {
		var format = function(str) {
			return str.toFixed(2);
		};
		
		var fragment = document.createDocumentFragment();
		
		if (this.head) {
			var head = document.createElement('div');
			head.innerHTML = "head("+format(this.head[0])+", "+format(this.head[1])+");";
			fragment.appendChild(head);
		}
		if (this.torso) {
			var torso = document.createElement('div');
			torso.innerHTML = "torso("+format(this.torso[0])+");";
			fragment.appendChild(torso);
		}
		if (this.left) {
			var left = document.createElement('div');
			left.innerHTML = "leftArm(";
			for (var i = 0; i < this.left.length-1; i++) {
				left.innerHTML+=format(this.left[i])+", ";
			}
			left.innerHTML+=format(this.left[this.left.length-1]) + ");\n";
			fragment.appendChild(left);
		}
		if (this.right) {
			var right = document.createElement('div');
			right.innerHTML = "rightArm(";
			for (var i = 0; i < this.right.length-1; i++) {
				right.innerHTML+=format(this.right[i])+", ";
			}
			right.innerHTML+=format(this.right[this.right.length-1]) + ");\n";
			fragment.appendChild(right);
		}	
		return fragment;
	}
	
});

museum.common.RobotPose.fromKorg = function(state) {
	pose = new museum.common.RobotPose();
	
	var mode = state.buttons[24];
	
	pose.setHead(-state.axes[9], -state.axes[0]);
	pose.setTorso(state.axes[9]);
	
	if (mode==0 || mode==1) {
		var pan = -state.axes[1];
		var lift = -state.axes[2];
		var upper_arm_roll = -state.axes[3];
		var elbow_flex = state.axes[4];
		var forearm_roll = -state.axes[5];
		var wrist_flex = state.axes[6];
		var wrist_roll = state.axes[16];
		var gripper = state.axes[7];
		pose.setLeft(pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper);
	}
	if (mode==0 || mode==2) {
		var pan = state.axes[1];
		var lift = -state.axes[2];
		var upper_arm_roll = state.axes[3];
		var elbow_flex = state.axes[4];
		var forearm_roll = state.axes[5];
		var wrist_flex = state.axes[6];
		var wrist_roll = state.axes[16];
		var gripper = state.axes[7];
		pose.setRight(pan, lift, upper_arm_roll, elbow_flex, forearm_roll, wrist_flex, wrist_roll, gripper);
	}	
	
	return pose;
};