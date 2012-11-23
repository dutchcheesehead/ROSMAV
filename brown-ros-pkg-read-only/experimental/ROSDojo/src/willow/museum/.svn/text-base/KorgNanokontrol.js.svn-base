dojo.provide("museum.KorgNanokontrol");

dojo.require("museum.KorgSlider");

dojo.declare("museum.KorgNanokontrol", [dijit._Widget, dijit._Templated], {
	
	// Internal variables
	templateString: dojo.cache("museum", "templates/KorgNanokontrol"),
	last_msg: {},
	
	postCreate: function() {
		this.headTiltSlider = this.addSlider();
		this.panSlider = this.addSlider();
		this.liftSlider = this.addSlider();
		this.upperSlider = this.addSlider();
		this.elbowSlider = this.addSlider();
		this.forearmSlider = this.addSlider();
		this.wristSlider = this.addSlider();
		this.gripSlider = this.addSlider();
		this.torsoSlider = this.addSlider();
		
		ros.subscribe("/korg_joy", dojo.hitch(this, "korgMessageReceived"), -1, "sensor_msgs/Joy");
	},
	
	addSlider: function(name) {
		var slider = new museum.KorgSlider();
		this.connect(slider, "onSliderMoved", "virtualKorgChanged");
		this.slidersAttach.appendChild(slider.domNode);
		return slider;
	},
	
	korgMessageReceived: function(msg) {
		this.last_msg = msg;
		this.headTiltSlider.setValue(msg.axes[0]);
		this.panSlider.setValue(msg.axes[1]);
		this.liftSlider.setValue(msg.axes[2]);
		this.upperSlider.setValue(msg.axes[3]);
		this.elbowSlider.setValue(msg.axes[4]);
		this.forearmSlider.setValue(msg.axes[5]);
		this.wristSlider.setValue(msg.axes[6]);
		this.gripSlider.setValue(msg.axes[7]);
		this.torsoSlider.setValue(msg.axes[8]);
	},
	
	virtualKorgChanged: function() {
		var msg = {};
		if (this.last_msg.axes) {
			msg.axes = this.last_msg.axes;
		} else {
			msg.axes = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
		}
		if (this.last_msg.buttons) {
			msg.buttons = this.last_msg.buttons;
		} else {
			msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
		}
		msg.axes[0] = this.headTiltSlider.value;
		msg.axes[1] = this.panSlider.value;
		msg.axes[2] = this.liftSlider.value;
		msg.axes[3] = this.upperSlider.value;
		msg.axes[4] = this.elbowSlider.value;
		msg.axes[5] = this.forearmSlider.value;
		msg.axes[6] = this.wristSlider.value;
		msg.axes[7] = this.gripSlider.value;
		msg.axes[8] = this.torsoSlider.value;
		console.log('publishing...')
		ros.publish("/korg_joy", "sensor_msgs/Joy", dojo.toJson(msg));
	}
	

});