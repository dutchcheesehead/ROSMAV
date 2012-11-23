dojo.provide("museum.PoseSequence");

dojo.require("dijit._Widget");
dojo.require("museum.PoseSequenceEntry");

dojo.declare("museum.PoseSequence", dijit._Widget, {
	
	// Internal variables
	poses: null,
	
	postCreate: function() {
		dojo.addClass(this.domNode, "pose-sequence");
		this.domNode.innerHTML = "&nbsp;"; // kinda a hack...
		
		this.poses = [];
	},
	
	addPose: function(korgState, imgPreview) {
		var pose = new museum.PoseSequenceEntry({ 
			imgPreview: imgPreview, 
			korgState: korgState,
			number: this.poses.length+1
		});
		this.domNode.appendChild(pose.domNode);
		this.poses.push(pose);
		dojo.window.scrollIntoView(pose.domNode);
	},
	
	setHeight: function(height) {
		console.log("Setting posesquence widget height to ", height);
		dojo.style(this.domNode, "height", height+"px");
	}
	
});