/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/


ros.visualization.Interactor = Class.extend({
  init: function(camera) 
  {
    this.camera = camera;
    this.width = 640;
    this.height = 480;
    this.interactorMatrix = sglIdentityM4();
  },
   
  keyDown : function(gl, keyCode, keyString) {
  },

  mouseMove : function(gl, x, y, mouseButtonsDown, keysDown) {
  },

  mouseWheel : function(gl, wheelDelta, x, y, mouseButtonsDown, keysDown) {
  },

  mouseDown: function(gl,mouseButtonsDown, x, y, pickedNode)
  {
  },

  mouseUp: function(gl, x, y, button, keysDown)
  {
  },

  resize : function(gl, width, height) {
    this.width = width;
    this.height = height;
  },
  
});

ros.include('visualization/interactor/orbitinteractor');
ros.include('visualization/interactor/puppetinteractor');
ros.include('visualization/interactor/selectinteractor');
ros.include('visualization/interactor/overlayinteractor');
