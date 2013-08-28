__author__ = 'Martin Felder, felder@in.tum.de'

from OpenGL.GL import * #@UnusedWildImport
from OpenGL.GLU import * #@UnusedWildImport
from OpenGL.GLUT import * #@UnusedWildImport

from math import sin, asin, cos, acos, pi, sqrt
from tools.mathhelpers import crossproduct, norm, dotproduct

import time
import Image #@UnresolvedImport

from pybrain.tools.networking.udpconnection import UDPClient


class ODEViewer(object):
    def __init__(self, servIP='127.0.0.1', ownIP='127.0.0.1', port='21590',
            buf='16384', verbose=False, window_name='ODE Viewer'):
        """Initialize

        Initializes all viewer attributes and starts the OpenGL engine.
        The UDP server is also started.
        """
        self.verbose = verbose
        self.window_name = window_name

        # initialize viewport starting size
        self.width = 800
        self.height = 600

        # Determine if fullscreen is active or not
        self.is_fullscreen = False

        self.aspect_ratio = float(self.width) / float(self.height)

        # initialize object which the camera follows
        self.mouseView = True

        # Toggles to determine if the view should zoom or rotate
        self.motion_rotate_mode = False
        self.motion_zoom_mode = False

        # Stores the location of the mouse the last time it was clicked
        self.motion_last_mouse_down_pos = [0.0, 0.0]

        self.centerObj = None

        self.cam_r_init = 1.0
        self.cam_theta_init = pi / 4.0
        self.cam_phi_init = 0.0

        self.cam_r = self.cam_r_init
        self.cam_theta = self.cam_theta_init
        self.cam_phi = self.cam_phi_init

        self.fps = 60
        self.dt = 1.0 / self.fps

        self.lasttime = time.time()
        self.starttime = time.time()

        self.capture_screen = False

        self.message = None

        # capture only every frameT. frame
        self.counter = 0
        self.frameT = 1

        self.init_gl()

        # set OpenGL callback functions
        glutKeyboardFunc(self._keyboard_callback)
        glutMouseFunc(self._mouse_callback)
        glutMotionFunc(self._motion_callback)
        glutPassiveMotionFunc(self._passive_motion_callback)
        glutDisplayFunc(self._display_callback)
        glutIdleFunc(self._idle_callback)

        # initialize udp client
        self.client = UDPClient(servIP, ownIP, port, buf, verbose=self.verbose)

        return


    def start(self):
        # start the OpenGL main loop
        glutMainLoop()
        return


    def set_dt(self, dt):
        self.dt = dt
        self.fps = 1.0 / self.dt
        return


    def set_fps(self, fps):
        self.fps = fps
        self.dt = 1.0 / self.fps
        return


    def init_gl(self):
        """ initialize OpenGL. This function has to be called only once before drawing. """
        glutInit([])

        # Open a window
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowPosition(0, 0)
        glutInitWindowSize(self.width, self.height)
        glutCreateWindow(self.window_name)

        # Initialize Viewport and Shading
        glViewport(0, 0, self.width, self.height)
        glShadeModel(GL_SMOOTH)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glClearColor(1.0, 1.0, 1.0, 0.0)

        # Initialize Depth Buffer
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        # Initialize Lighting
        glEnable(GL_LIGHTING)
        glLightfv(GL_LIGHT1, GL_AMBIENT, [0.5, 0.5, 0.5, 1.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT1, GL_POSITION, [0.0, 5.0, 5.0, 1.0])
        glEnable(GL_LIGHT1)

        # enable material coloring
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)

        glEnable(GL_NORMALIZE)
        return


    def prepare_gl(self):
        """Prepare drawing. This function is called in every step. It clears the screen and sets the new camera position"""
        # Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Calculate the current aspect ratio so we have an accurate model
        (_, _, x, y) = glGetIntegerv(GL_VIEWPORT)
        self.aspect_ratio = float(x) / float(y)

        # Projection mode
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.aspect_ratio, 0.1, 50.0)

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Center the camera on a given object or (0,0,0) if not specified
        if self.centerObj is not None:
            (centerX, centerY, centerZ) = self.centerObj.getPosition()
        else:
            centerX = centerY = centerZ = 0

        # Convert spherical to cartesian coordinates
        cam_x = self.cam_r * sin(self.cam_theta) * sin(self.cam_phi)
        cam_y = self.cam_r * cos(self.cam_theta)
        cam_z = self.cam_r * sin(self.cam_theta) * cos(self.cam_phi)

        # Place the camera and set the camera's target object
        gluLookAt(cam_x, cam_y, cam_z, centerX, centerY, centerZ, 0, 1, 0)

        return


    def draw_item(self, item):
        """ draws an object (spere, cube, plane, ...) """
        glDisable(GL_TEXTURE_2D)

        glPushMatrix()

        if item['type'] in ['GeomBox', 'GeomSphere', 'GeomCylinder', 'GeomCCylinder']:
            # set color of object (currently dark gray)
            if item.has_key('color'):
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glColor4f(*(item['color']))
            else: glColor3f(0.1, 0.1, 0.1)

            # transform (rotate, translate) body accordingly
            (x, y, z) = item['position']
            R = item['rotation']

            rot = [R[0], R[3], R[6], 0.0,
                   R[1], R[4], R[7], 0.0,
                   R[2], R[5], R[8], 0.0,
                      x,    y,    z, 1.0]

            glMultMatrixd(rot)

            # switch different geom objects
            if item['type'] == 'GeomBox':
                # cube
                (sx, sy, sz) = item['scale']
                glScaled(sx, sy, sz)
                glutSolidCube(1)
            elif item['type'] == 'GeomSphere':
                # sphere
                glutSolidSphere(item['radius'], 20, 20)

            elif item['type'] == 'GeomCCylinder':
                quad = gluNewQuadric()
                # draw cylinder and two spheres, one at each end
                glTranslate(0.0, 0.0, -item['length'] / 2)
                gluCylinder(quad, item['radius'], item['radius'], item['length'], 32, 32)
                glutSolidSphere(item['radius'], 20, 20)
                glTranslate(0.0, 0.0, item['length'])
                glutSolidSphere(item['radius'], 20, 20)

            elif item['type'] == 'GeomCylinder':
                glTranslate(0.0, 0.0, -item['length'] / 2)
                quad = gluNewQuadric()
                gluDisk(quad, 0, item['radius'], 32, 1)
                quad = gluNewQuadric()
                gluCylinder(quad, item['radius'], item['radius'], item['length'], 32, 32)
                glTranslate(0.0, 0.0, item['length'])
                quad = gluNewQuadric()
                gluDisk(quad, 0, item['radius'], 32, 1)
            else:
                # TODO: add other geoms here
                pass

        elif item['type'] == 'GeomPlane':
            # set color of plane (currently green)
            glColor3f(0.0, 0.2, 0.0)

            # for planes, we need a Quadric object
            quad = gluNewQuadric()
            gluQuadricTexture(quad, GL_TRUE)

            p = item['normal']      # the normal vector to the plane
            d = item['distance']    # the distance to the origin
            q = (0.0, 0.0, 1.0)     # the normal vector of default gluDisks (z=0 plane)

            # calculate the cross product to get the rotation axis
            c = crossproduct(p, q)
            # calculate the angle between default normal q and plane normal p
            theta = acos(dotproduct(p, q) / (norm(p) * norm(q))) / pi * 180

            # rotate the plane
            glPushMatrix()
            glTranslate(d * p[0], d * p[1], d * p[2])
            glRotate(-theta, c[0], c[1], c[2])
            gluDisk(quad, 0, 20, 20, 1)
            glPopMatrix()

        glPopMatrix()
        return


    @staticmethod
    def _loadTexture(textureFile):
        image = open(textureFile)
        ix = image.size[0]
        iy = image.size[1]

        image = image.tostring("raw", "RGBX", 0, -1)

        # Create Texture
        textures = glGenTextures(3)
        glBindTexture(GL_TEXTURE_2D, textures[0])       # 2d texture (x and y size)

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_BYTE, image)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)

        # Create Linear Filtered Texture
        glBindTexture(GL_TEXTURE_2D, textures[1])
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)

        # Create MipMapped Texture
        glBindTexture(GL_TEXTURE_2D, textures[2])
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST)
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, ix, iy, GL_RGBA, GL_UNSIGNED_BYTE, image)
        return textures


    def _display_callback (self):
        """ draw callback function """
        # Draw the scene
        self.prepare_gl()

        if self.message:
            for item in self.message:
                self.draw_item(item)

        glutSwapBuffers()

        if self.capture_screen:
            self._screenshot()

        return


    def _idle_callback(self):
        # Get the very latest data
        try:
            self.message = self.client.listen()
        except:
            pass

        # NOTE: This should be uncommented if real-time contraints are not
        # handled in the main loop of your application
        #t = self.dt - (time.time() - self.lasttime)
        #if (t > 0):
        #    time.sleep(t)
        #self.lasttime = time.time()

        # Display the latest data directly after receiving
        glutPostRedisplay()
        return


    def _keyboard_callback(self, key, x, y):
        """ keyboard call-back function. """
        if key == 's':
            self.capture_screen = not self.capture_screen
            print "Screen Capture: " + (self.capture_screen and "on" or "off")
        elif key == 'f':
            if self.is_fullscreen:
                # Toggle out of fullscreen mode
                glutReshapeWindow(self.width, self.height)
                glutPositionWindow(0, 0)
                glutSetCursor(GLUT_CURSOR_INHERIT)

                self.is_fullscreen = False
            else:
                # Toggle into fullscreen mode
                glutFullScreen()
                glutSetCursor(GLUT_CURSOR_NONE)
                self.is_fullscreen = True
        elif key in ['x', 'q']:
            sys.exit()
        elif key == 'c':
            print 'Reset Camera'
            self.cam_r = self.cam_r_init
            self.cam_theta = self.cam_theta_init
            self.cam_phi = self.cam_phi_init

        return


    def _mouse_callback(self, button, state, x, y):
        if state == GLUT_DOWN:
            self.motion_last_mouse_down_pos = (x, y)

        if button == GLUT_LEFT_BUTTON:
            # If the left button is down, activate rotation mode
            if state == GLUT_DOWN:
                self.motion_rotate_mode = True
            elif state == GLUT_UP:
                self.motion_rotate_mode = False
        elif button == GLUT_RIGHT_BUTTON:
            # If the right button is down, activate zoom mode
            if state == GLUT_DOWN:
                self.motion_zoom_mode = True
            elif state == GLUT_UP:
                self.motion_zoom_mode = False

        return


    def _motion_callback(self, x, y):
        x_down, y_down = self.motion_last_mouse_down_pos

        # Note that if a mouse has two buttons (left/right) pressed at the
        # same time, rotate mode will always override zoom mode
        if self.motion_rotate_mode:
            # Modify the polar and azimuthal angles via y, x mouse direction
            d_phi = 0.2 * (float(x) - float(x_down)) / self.height
            d_theta = 0.2 * (float(y) - float(y_down)) / self.width

            # Set camera angle constraints
            if 0.0 <= self.cam_theta + d_theta <= pi / 2.0:
                self.cam_theta += d_theta

            if -pi / 2.0 <= self.cam_phi + d_phi <= pi / 2.0:
                self.cam_phi += d_phi
        elif self.motion_zoom_mode:
            # Change radius from target based on the relative 'y' position of
            # the mouse from the last click
            dr = 0.2 * (float(y) - float(y_down)) / self.height

            # Limit the zoom range between 0.2 and 10 meters
            if 0.2 < self.cam_r + dr < 10:
                self.cam_r += dr

        return


    def _passive_motion_callback(self, x, z):
        pass


    def _screenshot(self, path_prefix='.', format='PNG'):
        """Saves a screenshot of the current frame buffer.
        The save path is <path_prefix>/.screenshots/shot<num>.png
        The path is automatically created if it does not exist.
        Shots are automatically numerated based on how many files
        are already in the directory."""

        if self.counter == self.frameT:
            self.counter = 1
            dir = os.path.join(path_prefix, 'screenshots')
            if not os.path.exists(dir):
                os.makedirs(dir)

            num_present = len(os.listdir(dir))
            num_digits = len(str(num_present))
            index = '0' * (5 - num_digits) + str(num_present)

            path = os.path.join(dir, 'shot' + index + '.' + format.lower())
            glPixelStorei(GL_PACK_ALIGNMENT, 1)
            data = glReadPixels(0, 0, self.width, self.height, GL_RGB, GL_UNSIGNED_BYTE)
            image = Image.fromstring("RGB", (self.width, self.height), data)
            image = image.transpose(Image.FLIP_TOP_BOTTOM)
            image.save(path, format)
            print 'Image saved to %s' % (os.path.basename(path))
        else:
            self.counter += 1

        return


if __name__ == '__main__':
    s = sys.argv[1:]
    odeview = ODEViewer(*s)
    odeview.start()
