#!/usr/bin/env python
import roslib
roslib.load_manifest('roswww')
import rospy

import subprocess

import tornado.ioloop
import tornado.web

def run(*args):
    '''run the provided command and return its stdout'''
    args = sum([(arg if type(arg) == list else [arg]) for arg in args], [])
    print ' '.join(args)
    return subprocess.Popen(args, stdout=subprocess.PIPE).communicate()[0].strip()

def split_words(text):
    '''return a list of lines where each line is a list of words'''
    return [line.strip().split() for line in text.split('\n')]

def get_packages():
    ''' Find the names and locations of all packages '''
    lines = split_words(run('rospack', 'list'))
    packages = [{ 'name': name, 'path': path } for name, path in lines]
    return packages

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        global packages
        self.write("<h1>ROS web server successfully started.</h1><h3>Package List</h3>")
        for package in packages:
            self.write("<div style='font-size: 10px'>"+package['name']+"</div>")
            
def create_webserver(packages):
    handlers = [(r"/", MainHandler)]
    
    for package in packages:
        handler = ("/"+package['name']+"/(.*)", tornado.web.StaticFileHandler, {"path": package['path']+"/www", "default_filename": "index.html" })
        handlers.append(handler)       
        
    print "Webserver initialized handling %d packages" % (len(packages), )
    application = tornado.web.Application(handlers)
    
    return application

def bind_webserver(application):    
    """ See if there's a default port, use 80 if not """
    default_port = rospy.get_param("http/default", 80)
    
    """ First, we try the default http port 80 """
    bound = bind_to_port(application, default_port)
    
    if not bound:
        """ Otherwise bind any available port within the specified range """
        start_port = rospy.get_param("http/range_start", 8000)
        end_port = rospy.get_param("http/range_end", 9000)
        bound = bind_in_range(application, start_port, end_port)
        
    return bound
        
def start_webserver(application):
    try: 
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print "Webserver shutting down"    

def bind_to_port(application, portno):
    print "Attempting to start webserver on port %d" % (portno, )
    try:
        application.listen(portno)
        print "Webserver successfully started on port %d" % (portno, )
    except Exception as inst:
        if inst.__module__=="socket" and inst.__class__.__name__=="error":
            if inst.errno==13:
                print "Insufficient priveliges to run as webserver on port %d. (%s)" % (portno, inst.strerror)
                print "-- Try re-running as super-user: sudo su; source ~/.bashrc)"
            if inst.errno==98:
                print "There is already a webserver running on port %d. (%s)" % (portno, inst.strerror)
                print "-- Try stopping your web server. For example, to stop apache: sudo /etc/init.d/apache2 stop"
        else:
            print "An error occurred attempting to listen on port %d: %s" % (portno, inst.strerror)
        return False
    return True
    
def bind_in_range(application, start_port, end_port):
    if (end_port > start_port):
        for i in range(start_port, end_port):
            if bind_to_port(application, i):
                return True
    return False

def run_webserver():
    packages = get_packages()
    server = create_webserver(packages)
    bound = bind_webserver(server)
    if (bound):
        start_webserver(server)
    else:
        print "Unable to bind webserver.  Exiting."

if __name__=='__main__':
    run_webserver()