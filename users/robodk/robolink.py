# This file contains two classes:
# Robolink()
# Item()
# These classes are the main objects used to create macros for RoboDK.
# An item is an object in the RoboDK tree (it can be either a robot, an object, a tool, a frame, a program, ...).

import struct
from robodk import *
from warnings import warn

# Tree item types
ITEM_TYPE_STATION=1
ITEM_TYPE_ROBOT=2
ITEM_TYPE_FRAME=3
ITEM_TYPE_TOOL=4
ITEM_TYPE_OBJECT=5
ITEM_TYPE_TARGET=6
ITEM_TYPE_PROGRAM=8
ITEM_TYPE_INSTRUCTION=9
ITEM_TYPE_PROGRAM_PYTHON=10
ITEM_TYPE_MACHINING=11
ITEM_TYPE_BALLBARVALIDATION=12
ITEM_TYPE_CALIBPROJECT=13
ITEM_TYPE_VALID_ISO9283=14

# Instruction types
INS_TYPE_INVALID = -1
INS_TYPE_MOVE = 0
INS_TYPE_MOVEC = 1
INS_TYPE_CHANGESPEED = 2
INS_TYPE_CHANGEFRAME = 3
INS_TYPE_CHANGETOOL = 4
INS_TYPE_CHANGEROBOT = 5
INS_TYPE_PAUSE = 6
INS_TYPE_EVENT = 7
INS_TYPE_CODE = 8
INS_TYPE_PRINT = 9

# Move types
MOVE_TYPE_INVALID = -1
MOVE_TYPE_JOINT = 1
MOVE_TYPE_LINEAR = 2
MOVE_TYPE_CIRCULAR = 3

# Station parameters request
PATH_OPENSTATION = 'PATH_OPENSTATION'
FILE_OPENSTATION = 'FILE_OPENSTATION'
PATH_DESKTOP = 'PATH_DESKTOP'

# Script execution types
RUNMODE_SIMULATE=1                      # performs the simulation moving the robot (default)
RUNMODE_QUICKVALIDATE=2                 # performs a quick check to validate the robot movements
RUNMODE_MAKE_ROBOTPROG=3                # makes the robot program
RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD=4     # makes the robot program and updates it to the robot
RUNMODE_MAKE_ROBOTPROG_AND_START=5      # makes the robot program and starts it on the robot (independently from the PC)
RUNMODE_RUN_ROBOT=6                     # moves the real robot from the PC (PC is the client, the robot behaves like a server)

# Program execution type
PROGRAM_RUN_ON_SIMULATOR=1  # Set the program to run on the simulator
PROGRAM_RUN_ON_ROBOT=2      # Set the program to run on the robot

# Robot connection status
ROBOTCOM_PROBLEMS       = -3
ROBOTCOM_DISCONNECTED   = -2
ROBOTCOM_NOT_CONNECTED  = -1
ROBOTCOM_READY          = 0
ROBOTCOM_WORKING        = 1
ROBOTCOM_WAITING        = 2
ROBOTCOM_UNKNOWN        = -1000

# TCP calibration types
CALIBRATE_TCP_BY_POINT = 0
CALIBRATE_TCP_BY_PLANE = 1

# projection types (for AddCurve)
PROJECTION_NONE                = 0 # No curve projection
PROJECTION_CLOSEST             = 1 # The projection will the closest point on the surface
PROJECTION_ALONG_NORMAL        = 2 # The projection will be done along the normal.
PROJECTION_ALONG_NORMAL_RECALC = 3 # The projection will be done along the normal. Furthermore, the normal will be recalculated according to the surface normal.

# Euler type
EULER_RX_RYp_RZpp = 0 # generic
EULER_RZ_RYp_RXpp = 1 # ABB RobotStudio
EULER_RZ_RYp_RZpp = 2 # Kawasaki, Adept, Staubli
EULER_RZ_RXp_RZpp = 3 # CATIA, SolidWorks
EULER_RX_RY_RZ    = 4 # Fanuc, Kuka, Motoman, Nachi
EULER_RZ_RY_RX    = 5 # CRS
EULER_QUEATERNION = 6 # ABB Rapid

# State of the RoboDK window
WINDOWSTATE_HIDDEN      = -1
WINDOWSTATE_SHOW        = 0
WINDOWSTATE_MINIMIZED   = 1
WINDOWSTATE_NORMAL      = 2
WINDOWSTATE_MAXIMIZED   = 3
WINDOWSTATE_FULLSCREEN  = 4
WINDOWSTATE_CINEMA      = 5
WINDOWSTATE_FULLSCREEN_CINEMA= 6

# Instruction program call type:
INSTRUCTION_CALL_PROGRAM = 0
INSTRUCTION_INSERT_CODE = 1
INSTRUCTION_START_THREAD = 2
INSTRUCTION_COMMENT = 3

 
class Robolink:
    """This class is the link to allows to create macros and automate Robodk.
    Any interaction is made through \"items\" (Item() objects). An item is an object in the
    robodk tree (it can be either a robot, an object, a tool, a frame, a 
    program, ...)."""
    APPLICATION_DIR = 'C:/RoboDK/bin/RoboDK.exe'    # file path to the robodk program (executable)
    SAFE_MODE = 1           # checks that provided items exist in memory
    AUTO_UPDATE = 0         # if AUTO_UPDATE is zero, the scene is rendered after every function call
    TIMEOUT = 10             # timeout for communication, in seconds
    COM = None              # tcpip com
    IP = 'localhost'        # IP address of the simulator (localhost if it is the same computer), otherwise, use RL = Robolink('yourip') to set to a different IP
    PORT_START = 20500      # port to start looking for app connection
    PORT_END = 20500        # port to stop looking for app connection
    PORT = -1
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _is_connected(self):
        "Returns 1 if connection is valid, returns 0 if connection is invalid"
        if not self.COM: return 0
        connected = 1        
        #try:
        #    self.COM.settimeout(0)
        #    check = self.COM.recv(1)
        #except:
        #    connected = 0
        #    
        #self.COM.settimeout(self.TIMEOUT)
        return connected

    def _check_connection(self):
        """If we are not connected it will attempt a connection, if it fails, it will throw an error"""
        if not self._is_connected() and self.Connect() < 1:
            raise Exception('Unable to connect')
        #To do: Clear input buffer.

    def _check_status(self):
        """This function checks the status of the connection"""
        status = self._rec_int()
        if status > 0 and status < 10:
            strproblems = 'Unknown error'
            if status == 1:
                strproblems = 'Invalid item provided: The item identifier provided is not valid or it does not exist.'
            elif status == 2: #output warning
                strproblems = self._rec_line()
                print('WARNING: ' + strproblems)
                #warn(strproblems)# does not show where is the problem...
                return 0
            elif status == 3: #output error
                strproblems = self._rec_line()
                raise Exception(strproblems)
            elif status == 9:
                strproblems = 'Invalid license. Contact us at: www.robodk.com'
            print(strproblems)
            raise Exception(strproblems)
        elif status == 0:
            # everything is OK
            status = status;
        else:
            raise Exception('Problems running function')
        return status

    def _check_color(self, color):
        """Formats the color in a vector of size 4x1 and ranges [0,1]"""
        if not isinstance(color,list) or len(color) < 3 or len(color) > 4:
            raise Exception('The color vector must be a list of 3 or 4 values')
        if len(color) == 3:
            color.append(1)
        if max(color) > 1 or min(color) < -1:
            print("WARNING: Color provided is not in the range [0,1] ([r,g,b,a])")
        return color

    def _send_line(self, string=None):
        """Sends a string of characters with a \\n"""
        self.COM.send(bytes(string+'\n','utf-8')) # Python 3.x only
        #self.COM.send(bytes(string+'\n')) # Python 2.x only

    def _rec_line(self):
        """Receives a string. It reads until if finds LF (\\n)"""
        string = ''
        chari = self.COM.recv(1).decode('utf-8')
        while chari != '\n':    # read until LF
            string = string + chari
            chari = self.COM.recv(1).decode('utf-8')
        return str(string) # python 2 and python 3 compatible

    def _send_item(self, item):
        """Sends an item pointer"""
        if isinstance(item, Item):
            self.COM.send(struct.pack('>Q',item.item))#q=unsigned long long (64 bits), d=float64
            return
        self.COM.send(struct.pack('>Q',item))#q=unsigned long long (64 bits), d=float64

    def _rec_item(self):
        """Receives an item pointer"""
        buffer = self.COM.recv(8)
        item = struct.unpack('>Q',buffer)#q=unsigned long long (64 bits), d=float64
        buffer2 = self.COM.recv(4)
        itemtype = struct.unpack('>i',buffer2)
        return Item(self,item[0], itemtype[0])
        
    def _send_ptr(self, ptr_h):
        """Sends a generic pointer"""
        self.COM.send(struct.pack('>Q',ptr_h))#q=unsigned long long (64 bits), d=float64

    def _rec_ptr(self):
        """Receives a generic pointer"""
        buffer = self.COM.recv(8)
        ptr_h = struct.unpack('>Q',buffer)#q=unsigned long long (64 bits), d=float64
        return ptr_h

    def _send_pose(self, pose):
        """Sends a pose (4x4 matrix)"""
        if not pose.isHomogeneous():
            print("Warning: pose is not homogeneous!")
            print(pose)
        posebytes = b''
        for j in range(4):
            for i in range(4):
                posebytes = posebytes + struct.pack('>d',pose[i,j])
        self.COM.send(posebytes)

    def _rec_pose(self):
        """Receives a pose (4x4 matrix)"""
        posebytes = self.COM.recv(16*8)
        posenums = struct.unpack('>16d',posebytes)
        pose = Mat(4,4)
        cnt = 0
        for j in range(4):
            for i in range(4):
                pose[i,j] = posenums[cnt]
                cnt = cnt + 1
        return pose
        
    def _send_xyz(self, pos):
        """Sends an xyz vector"""
        posbytes = b''
        for i in range(3):
            posbytes = posbytes + struct.pack('>d',pos[i])
        self.COM.send(posbytes)

    def _rec_xyz(self):
        """Receives an xyz vector"""
        posbytes = self.COM.recv(3*8)
        posnums = struct.unpack('>3d',posbytes)
        pos = [0,0,0]
        for i in range(3):
            pos[i] = posnums[i]
        return pos

    def _send_int(self, num):
        """Sends an int (32 bits)"""
        if isinstance(num, float):
            num = round(num)
        elif not isinstance(num, int):
            num = num[0]
        self.COM.send(struct.pack('>i',num))

    def _rec_int(self):
        """Receives an int (32 bits)"""
        buffer = self.COM.recv(4)
        num = struct.unpack('>i',buffer)
        return num[0]

    def _send_array(self, values):
        """Sends an array of doubles"""
        if not isinstance(values,list):#if it is a Mat() with joints
            values = (values.tr()).rows[0];          
        nval = len(values)
        self._send_int(nval)        
        if nval > 0:
            buffer = b''
            for i in range(nval):
                buffer = buffer + struct.pack('>d',values[i])
            self.COM.send(buffer)

    def _rec_array(self):
        """Receives an array of doubles"""
        nvalues = self._rec_int()
        if nvalues > 0:
            buffer = self.COM.recv(8*nvalues)
            values = list(struct.unpack('>'+str(nvalues)+'d',buffer))
            #values = fread(self.COM, nvalues, 'double')
        else:
            values = [0]
        return Mat(values)

    def _send_matrix(self, mat):
        """Sends a 2 dimensional matrix (nxm)"""
        size = mat.size()
        self._send_int(size[0])
        self._send_int(size[1])
        for j in range(size[1]):
            matbytes = b''
            for i in range(size[0]):
                matbytes = matbytes + struct.pack('>d',mat[i,j])
            self.COM.send(matbytes)

    def _rec_matrix(self):
        """Receives a 2 dimensional matrix (nxm)"""
        size1 = self._rec_int()
        size2 = self._rec_int()
        recvsize = size1*size2*8
        BUFFER_SIZE = 512
        if recvsize > 0:
            matbytes = b''
            to_receive = min(recvsize, BUFFER_SIZE)
            while to_receive > 0:
                matbytes += self.COM.recv(to_receive)
                to_receive = min(recvsize - len(matbytes), BUFFER_SIZE)
            matnums = struct.unpack('>'+str(size1*size2)+'d',matbytes)
            mat = Mat(size1,size2)
            cnt = 0
            for j in range(size2):
                for i in range(size1):
                    mat[i,j] = matnums[cnt]
                    cnt = cnt + 1
        else:
            mat = Mat([[]])
        return mat

    def _moveX(self, target, itemrobot, movetype, blocking=True):
        """Performs a linear or joint movement. Use MoveJ or MoveL instead."""
        #self._check_connection();
        itemrobot.WaitMove()# checks connection
        command = 'MoveX'
        self._send_line(command)
        self._send_int(movetype)
        if isinstance(target,Item):# target is an item
            self._send_int(3)
            self._send_array([])
            self._send_item(target)
        elif isinstance(target,list) or target.size() != (4,4):# target are joints
            self._send_int(1)
            self._send_array(target)
            self._send_item(0)
        elif target.size() == (4,4):    # target is a pose
            self._send_int(2)
            mattr = target.tr()
            self._send_array(mattr.rows[0]+mattr.rows[1]+mattr.rows[2]+mattr.rows[3])
            self._send_item(0)
        else:
            raise Exception('Invalid input values')
        self._send_item(itemrobot)
        self._check_status()
        if blocking:
            itemrobot.WaitMove()
            
    def MoveC(self, target1, target2, itemrobot, blocking=True):
        """Performs a linear or joint movement. Use MoveJ or MoveL instead."""
        #self._check_connection();
        itemrobot.WaitMove()# checks connection
        command = 'MoveC'
        self._send_line(command)
        self._send_int(3)
        if isinstance(target1,Item):# target1 is an item
            self._send_int(3)
            self._send_array([])
            self._send_item(target1)
        elif isinstance(target1,list) or target1.size() != (4,4):# target1 are joints
            self._send_int(1)
            self._send_array(target1)
            self._send_item(0)
        elif target1.size() == (4,4):    # target1 is a pose
            self._send_int(2)
            mattr = target1.tr()
            self._send_array(mattr.rows[0]+mattr.rows[1]+mattr.rows[2]+mattr.rows[3])
            self._send_item(0)
        else:
            raise Exception('Invalid input value for target 1')
        if isinstance(target2,Item):# target1 is an item
            self._send_int(3)
            self._send_array([])
            self._send_item(target2)
        elif isinstance(target2,list) or target2.size() != (4,4):# target2 are joints
            self._send_int(1)
            self._send_array(target2)
            self._send_item(0)
        elif target2.size() == (4,4):    # target2 is a pose
            self._send_int(2)
            mattr = target2.tr()
            self._send_array(mattr.rows[0]+mattr.rows[1]+mattr.rows[2]+mattr.rows[3])
            self._send_item(0)
        else:
            raise Exception('Invalid input value for target 2')
        self._send_item(itemrobot)
        self._check_status()
        if blocking:
            itemrobot.WaitMove()

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    def __init__(self, robodk_ip='localhost', port=None):
        """A connection is attempted upon creation of the object"""
        self.IP = robodk_ip
        if port is not None:
            self.PORT_START = port
            self.PORT_END = port
        self.Connect()

    def _set_connection_params(self, safe_mode=1, auto_update=0, timeout=None):
        """Sets some behavior parameters: SAFE_MODE, AUTO_UPDATE and TIMEOUT.
        SAFE_MODE checks that item pointers provided by the user are valid.
        AUTO_UPDATE checks that item pointers provided by the user are valid.
        TIMEOUT is the timeout to wait for a response. Increase if you experience problems loading big files.
        If connection failed returns 0.
        In  1 (optional) : int -> SAFE_MODE (1=yes, 0=no)
        In  2 (optional) : int -> AUTO_UPDATE (1=yes, 0=no)
        In  3 (optional) : int -> TIMEOUT (1=yes, 0=no)
        Out 1 : int -> connection status (1=ok, 0=problems)
        Example:
            _set_connection_params(0,0); # Use for speed. Render() must be called to refresh the window.
            _set_connection_params(1,1); # Default behavior. Updates every time."""
        self.SAFE_MODE = safe_mode
        self.AUTO_UPDATE = auto_update
        self.TIMEOUT = timeout or self.TIMEOUT
        self._send_line('CMD_START')
        self._send_line(str(self.SAFE_MODE) + ' ' + str(self.AUTO_UPDATE))
        #fprintf(self.COM, sprintf('%i %i'), self.SAFE_MODE, self.AUTO_UPDATE))# appends LF
        response = self._rec_line()
        if response == 'READY':
            ok = 1
        else:
            ok = 0
        return ok

    def Connect(self):
        """Establishes a connection with robodk. robodk must be running, otherwise, the variable APPLICATION_DIR must be set properly.
        If the connection succeededs it returns 1, otherwise it returns 0"""
        import socket
        connected = 0
        for i in range(2):
            for port in range(self.PORT_START,self.PORT_END+1):
                self.COM = socket.socket(socket.AF_INET, socket.SOCK_STREAM)#'localhost', port, 'Timeout', self.TIMEOUT, 'BytesAvailableFcnMode', 'byte', 'InputBufferSize', 4000); 
                #self.COM.setblocking(1) #default is blocking
                self.COM.settimeout(1)
                try:
                    self.COM.connect((self.IP, port))                
                    connected = self._is_connected()
                    if connected > 0:
                        self.COM.settimeout(self.TIMEOUT)
                        break
                except:
                    connected = connected

            if connected > 0:# if status is closed, try to open application
                self.PORT = port
                break;
            else:
                if self.IP != 'localhost':
                    break;
                    
                try:
                    print('Starting %s\n' % self.APPLICATION_DIR)
                    import subprocess
                    subprocess.Popen(self.APPLICATION_DIR)
                    #import os # blocking
                    #os.system(self.APPLICATION_DIR)
                    #sample: os.system(r"C:\Documents and Settings\flow_model\flow.exe")
                    #winopen(self.APPLICATION_DIR);
                    import time
                    time.sleep(5) # wat for RoboDK to start and check network license.
                except:
                    raise Exception('Application path is not correct or could not start: ' + self.APPLICATION_DIR)

        if connected > 0 and not self._set_connection_params():
            connected = 0
        return connected

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # public methods
    def Item(self, name, itemtype=None):
        """Returns an item by its name. If there is no exact match it will return the last closest match.
        Specify what type of item you are looking for with itemtype. This is useful if 2 items have the same name but different type.
        (check variables ITEM_TYPE_*)
        Example:
            RL = Robolink()
            item = RL.Get_Item('Robot')
            item2 = RL.Get_Item('Robot', ITEM_TYPE_ROBOT)"""
        self._check_connection()
        if itemtype is None:
            command = 'G_Item'
            self._send_line(command)
            self._send_line(name)
        else:
            command = 'G_Item2'
            self._send_line(command)
            self._send_line(name)
            self._send_int(itemtype)
        item = self._rec_item()#     item = fread(com, 2, 'ulong');% ulong is 32 bits!!!
        self._check_status()
        return item


    def ItemList(self, filter=None, list_names=True):
        """Returns a list of items (list of name or pointers) of all available items in the currently open station in robodk.
        Optionally, use a filter to return specific items (example: ItemList(filter = ITEM_CASE_ROBOT))
        Optionally, use a list_names=False to return pointers (example: ItemList(filter = ITEM_CASE_ROBOT, list_names = False))"""
        self._check_connection()
        retlist = []
        if list_names:
            if filter is None:
                command = 'G_List_Items'
                self._send_line(command)
            else:
                command = 'G_List_Items_Type'
                self._send_line(command)
                self._send_int(filter)
            count = self._rec_int()
            for i in range(count):
                namei = self._rec_line()
                retlist.append(namei)
        else:
            if filter is None:
                command = 'G_List_Items_ptr'
                self._send_line(command)
            else:
                command = 'G_List_Items_Type_ptr'
                self._send_line(command)
                self._send_int(filter)
            count = self._rec_int()
            for i in range(count):
                itemi = self._rec_item()
                retlist.append(itemi)
        self._check_status()
        return retlist

    def ItemUserPick(self, message="Pick one item", itemtype=None):
        """Shows a RoboDK popup to select one object from the open station.
        An item type can be specified to filter desired items. If no type is specified, all items are selectable.
        (check variables ITEM_TYPE_*)
        Example:
           RL.ItemUserPick("Pick a robot", ITEM_TYPE_ROBOT)"""
        self._check_connection()
        if itemtype is None:
            itemtype = -1
        command = 'PickItem'
        self._send_line(command)
        self._send_line(message)
        self._send_int(itemtype)
        self.COM.settimeout(3600) # wait up to 1 hour for user input
        item = self._rec_item()
        self.COM.settimeout(self.TIMEOUT)
        self._check_status()
        return item

    def ShowRoboDK(self):
        """Shows or raises the RoboDK window"""
        self._check_connection()
        command = 'RAISE'
        self._send_line(command)
        self._check_status()
        
    def HideRoboDK(self):
        """Hides the RoboDK window"""
        self._check_connection()
        command = 'HIDE'
        self._send_line(command)
        self._check_status()
        
    def setWindowState(self, windowstate=WINDOWSTATE_NORMAL):
        """Set the state of the RoboDK window"""
        self._check_connection()
        command = 'S_WindowState'
        self._send_line(command)
        self._send_int(windowstate)
        self._check_status()
    
    def ShowMessage(self, message, popup=True):
        """Shows a message on the RoboDK window. By default, the message will be a blocking popup. Alternatively, it can be a message displayed at the bottom of RoboDK's main window."""
        self._check_connection()
        if popup:
            command = 'ShowMessage'
            self._send_line(command)
            self._send_line(message)
            self.COM.settimeout(3600) # wait up to 1 hour user to hit OK
            self._check_status()
            self.COM.settimeout(self.TIMEOUT)
        else:
            command = 'ShowMessageStatus'
            self._send_line(command)
            self._send_line(message)
            self._check_status()
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def Copy(self, item):
        """Makes a copy of an item (same as Ctrl+C), which can be pasted (Ctrl+V) using Paste_Item().
        In 1 : item
        Example:
            RL = Robolink()
            object = RL.Item('My Object');
            object.Copy()         #RL.Copy(object); also works
            newobject = RL.Paste();
            newobject.setName('My Object (copy 1)');
            newobject = RL.Paste();
            newobject.setName('My Object (copy 2)');"""
        self._check_connection()
        command = 'Copy'
        self._send_line(command)
        self._send_item(item)
        self._check_status()

    def Paste(self, toparent=0):
        """Pastes the copied item (same as Ctrl+V). Needs to be used after Copy_Item(). See Copy_Item() for an example.
        In 1 (optional): item -> parent to paste to"""
        self._check_connection()
        command = 'Paste'
        self._send_line(command)
        self._send_item(toparent)
        newitem = self._rec_item()
        self._check_status()
        return newitem

    def AddFile(self, filename, parent=0):
        """Loads a file and attaches it to parent. It can be any file supported by robodk.
        Timeout may have to be increased if big files are loaded.
        In 1  : string -> absolute path of the file
        In 2 (optional): item -> parent to attach
        Out 1 : item -> added item (0 if failed)
        Example:
            RL = Robolink()
            item = Add_File(r'C:\\Users\\Name\\Desktop\\object.step')
            RL.Set_Pose(item, transl(100,50,500))"""
        self._check_connection()
        command = 'Add'
        self._send_line(command)
        self._send_line(filename)
        self._send_item(parent)
        newitem = self._rec_item()
        self._check_status()
        return newitem
        
    def AddShape(self, triangle_points, add_to=0):
        """Adds a shape provided triangle coordinates. Triangles must be provided as a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3 because vertices must be stacked by groups of 3. Each group is a triangle.
        In 2 (optional): item -> object to add the shape
        Out 1 : item -> added object/shape (0 if failed)"""
        if isinstance(triangle_points,list):
            triangle_points = tr(Mat(triangle_points))
        elif not isinstance(triangle_points, Mat):
            raise Exception("triangle_points must be a 3xN or 6xN list or matrix")
        self._check_connection()
        command = 'AddShape'
        self._send_line(command)
        self._send_matrix(triangle_points)
        self._send_item(add_to)
        newitem = self._rec_item()
        self._check_status()
        return newitem    
        
    def AddCurve(self, curve_points, reference_object=0, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Adds a curve provided point coordinates. The provided points must be a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3
        In 2 (optional): reference_object -> object to add the curve and/or project the curve to the surface
        In 3 (optional): add_to_ref -> If True, the curve will be added as part of the object in the RoboDK item tree (a reference object must be provided)
        In 4 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.
        Out 1 : item -> added object/curve (0 if failed)"""
        if isinstance(curve_points,list):
            curve_points = Mat(curve_points).tr()
        elif not isinstance(curve_points, Mat):
            raise Exception("curve_points must be a 3xN or 6xN list or matrix")
        self._check_connection()
        command = 'AddWire'
        self._send_line(command)
        self._send_matrix(curve_points)
        self._send_item(reference_object)
        self._send_int(1 if add_to_ref else 0)
        self._send_int(projection_type)        
        newitem = self._rec_item()
        self._check_status()
        return newitem   
        
    def AddPoints(self, points, reference_object=0, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Adds a list of points to an object. The provided points must be a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3
        In 2 (optional): reference_object -> object to add the curve and/or project the curve to the surface
        In 3 (optional): add_to_ref -> If True, the curve will be added as part of the object in the RoboDK item tree (a reference object must be provided)
        In 4 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.
        Out 1 : item -> added object/curve (0 if failed)"""
        if isinstance(points,list):
            points = Mat(points).tr()
        elif not isinstance(points, Mat):
            raise Exception("points must be a 3xN or 6xN list or matrix")
        self._check_connection()
        command = 'AddPoints'
        self._send_line(command)
        self._send_matrix(points)
        self._send_item(reference_object)
        self._send_int(1 if add_to_ref else 0)
        self._send_int(projection_type)        
        newitem = self._rec_item()
        self._check_status()
        return newitem   

    def ProjectPoints(self, points, object_project, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Projects a point given its coordinates. The provided points must be a list of [XYZ] coordinates. Optionally, a vertex normal can be provided [XYZijk].
        In 1  : matrix 3xN or 6xN -> list of points to project
        In 2  : object_project -> object to project
        In 3 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.
        Out 1 : item -> projected points (empty matrix if failed)"""
        islist = False
        if isinstance(points,list):
            islist = True
            points = Mat(points)
        elif not isinstance(points, Mat):
            raise Exception("points must be a 3xN or 6xN list or matrix")
        self._check_connection()
        command = 'ProjectPoints'
        self._send_line(command)
        self._send_matrix(points)
        self._send_item(object_project)
        self._send_int(projection_type)        
        projected_points = self._rec_matrix()
        self._check_status()
        if islist:
            projected_points = projected_points.tolist()
        return projected_points           
        
    def Save(self, filename, itemsave=0):
        """Save an item to a file. If no item is provided, the open station is saved."""
        self._check_connection()
        command = 'Save'
        self._send_line(command)
        self._send_line(filename)
        self._send_item(itemsave)
        self._check_status()

    def AddTarget(self, name, itemparent=0, itemrobot=0):
        """Adds a new target that can be reached with a robot.
        In  1 : string -> name of the target
        In  2 (optional): item -> parent to attach to (such as a frame)
        In  3 (optional): item -> main robot that will be used to go to self target
        Out 1 : item -> the new item created"""
        self._check_connection()
        command = 'Add_TARGET'
        self._send_line(command)
        self._send_line(name)
        self._send_item(itemparent)
        self._send_item(itemrobot)
        newitem = self._rec_item()
        self._check_status()
        return newitem

    def AddFrame(self, name, itemparent=0):
        """Adds a new Frame that can be referenced by a robot.
        In  1 : string -> name of the frame
        In  2 (optional): item -> parent to attach to (such as the rrobot base frame)
        Out 1 : item -> the new item created"""
        self._check_connection()
        command = 'Add_FRAME'
        self._send_line(command)
        self._send_line(name)
        self._send_item(itemparent)
        newitem = self._rec_item()
        self._check_status()
        return newitem

    def AddProgram(self, name, itemrobot=0):
        """Adds a new program.
        In  1 : string -> name of the program
        In  2 (optional): item -> robot that will be used
        Out 1 : item -> the new item created"""
        self._check_connection()
        command = 'Add_PROG'
        self._send_line(command)
        self._send_line(name)
        self._send_item(itemrobot)
        newitem = self._rec_item()
        self._check_status()
        return newitem
        
    def AddMillingProject(self, name='Milling settings', itemrobot=0):
        """Adds a new machining project. Machining projects can also be used for 3D printing, curve following and point following.
        In  1 : string -> name of the project
        In  2 (optional): item -> robot that will be used
        Out 1 : item -> the new item created"""
        self._check_connection()
        command = 'Add_MACHINING'
        self._send_line(command)
        self._send_line(name)
        self._send_item(itemrobot)
        newitem = self._rec_item()
        self._check_status()
        return newitem

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def RunProgram(self, fcn_param, wait_for_finished = False):
        """Adds a function call in the program output. RoboDK will handle the syntax when the code is generated for a specific robot. If the program exists it will also run the program in simulate mode.
        In  1 : fcn call -> string of the program to run
        Out 1 : this function always returns 0"""        
        if wait_for_finished:
            prog_item = self.Item(fcn_param, ITEM_TYPE_PROGRAM)
            if not prog_item.Valid():
                raise Exception('Invalid program %s' % fcn_param)
            prog_status = prog_item.RunProgram()
            prog_item.WaitFinished()
        else:
            prog_status = self.RunCode(fcn_param, True)
        return prog_status
    
    def RunCode(self, code, code_is_fcn_call=False):
        """Adds code to run in the program output. If the program exists it will also run the program in simulate mode.
        In  1 : code -> string of the code or program to run
        In  2 : code_is_fcn_call -> True if the code corresponds to a function call (same as RunProgram), if so, RoboDK will handle the syntax when the code is generated for a specific robot
        Out 1 : this function always returns 0"""
        self._check_connection()
        command = 'RunCode'
        self._send_line(command)
        self._send_int(code_is_fcn_call)
        self._send_line(code.replace('\r\n','<<br>>').replace('\n','<<br>>'))
        prog_status = self._rec_int()
        self._check_status()
        return prog_status
    
    def RunMessage(self, message, message_is_comment=False):
        """Shows a message or a comment in the output robot program.
        In  1 : string -> message or comment to show in the teach pendant
        Out 1 : int -> if message_is_comment is set to True (or 1) the message will appear only as a comment in the code"""
        print('Message: ' + message)
        self._check_connection()
        command = 'RunMessage'
        self._send_line(command)
        self._send_int(message_is_comment)
        self._send_line(message.replace('\r\n','<<br>>').replace('\n','<<br>>'))
        self._check_status()    

    def Render(self, always_render=False):
        """Renders the scene. This function turns off rendering unless always_render is set to true."""
        auto_render = not always_render;
        self._check_connection()
        command = 'Render'
        self._send_line(command)
        self._send_int(auto_render)
        self._check_status()

    def IsInside(self, object_inside, object):
        """Returns (1/True) if object_inside is inside the object"""
        self._check_connection()
        self._send_line('IsInside')
        self._send_item(object_inside)
        self._send_item(object)        
        inside = self._rec_int()
        self._check_status()
        return inside    
        
    def Collisions(self):
        """Returns the number of pairs of objects that are currently in a collision state."""
        self._check_connection()
        command = 'Collisions'
        self._send_line(command)
        ncollisions = self._rec_int()
        self._check_status()
        return ncollisions
        
    def Collision(self, item1, item2):
        """Returns 1 if item1 and item2 collided. Otherwise returns 0."""
        self._check_connection()
        command = 'Collided'
        self._send_line(command)
        self._send_item(item1)
        self._send_item(item2)        
        ncollisions = self._rec_int()
        self._check_status()
        return ncollisions

    def setSimulationSpeed(self, speed):
        """Sets the current simulation speed. Set the speed to 1 for a real-time simulation. The slowest speed allowed is 0.001 times the real speed. Set to a high value (>100) for fast simulation results.""" 
        self._check_connection()
        command = 'SimulateSpeed'
        self._send_line(command)
        self._send_int(speed*1000)
        self._check_status()
        
    def SimulationSpeed(self):
        """Gets the current simulation speed. Set the speed to 1 for a real-time simulation.""" 
        self._check_connection()
        command = 'GetSimulateSpeed'
        self._send_line(command)
        speed = self._rec_int()/1000.0
        self._check_status()
        return speed
    
    def setRunMode(self, run_mode=1):
        """Sets the behavior of the script. By default, robodk shows the path simulation for movement instructions (run_mode=1).
        Setting the run_mode to 2 allows to perform a quick check to see if the path is feasible.
        In   1 : int = RUNMODE
        RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        ...
        if robot.Connect() is used, RUNMODE_RUN_FROM_PC is selected automatically.
        """
        self._check_connection()
        command = 'S_RunMode'
        self._send_line(command)
        self._send_int(run_mode)
        self._check_status()
        
    def RunMode(self):
        """Returns the behavior of the script. By default, robodk shows the path simulation for movement instructions (run_mode=1).
        If run_mode = 2, the script is performing a quick check to see if the path is feasible (usually managed by the GUI).
        If run_mode = 3, the script is generating the robot program (usually managed by the GUI).
        Out  1 : int = RUNMODE
        RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        RUNMODE_RUN_REAL=4        moves the real robot is it is connected
        """            
        self._check_connection()
        command = 'G_RunMode'
        self._send_line(command)
        runmode = self._rec_int()
        self._check_status()
        return runmode

    def getParams(self):
        """Gets all the user parameters from the open RoboDK station.
        The parameters can also be modified by right clicking the station and selecting "shared parameters"
        Out 1 : list of pairs of strings
        User parameters can be added or modified by the user"""    
        self._check_connection()
        command = 'G_Params'
        self._send_line(command)
        nparam = self._rec_int()
        params = []
        for i in range(nparam):
            param = self._rec_line()
            value = self._rec_line()
            try:
                value = float(value) # automatically convert int, long and float
            except ValueError:
                value = value
            params.append([param, value])
        self._check_status()
        return params
        
    def getParam(self, param='PATH_OPENSTATION'):
        """Gets a global or a user parameter from the open RoboDK station.
        The parameters can also be modified by right clicking the station and selecting "shared parameters"
        In  1 : string = parameter
        Out 1 : string = value
        Available parameters:
        PATH_OPENSTATION = folder path of the current .stn file
        FILE_OPENSTATION = file path of the current .stn file
        PATH_DESKTOP = folder path of the user's folder
        Other parameters can be added or modified by the user"""    
        self._check_connection()
        command = 'G_Param'
        self._send_line(command)
        self._send_line(param)
        value = self._rec_line()
        self._check_status()
        if value.startswith('UNKNOWN '):
            return None
        try:
            return float(value) # automatically convert int, long and float
        except ValueError:
            return value
        
    def setParam(self, param, value):
        """Sets a global parameter from the RoboDK station. If the parameters exists, it will be modified. If not, it will be added to the station.
        The parameters can also be modified by right clicking the station and selecting "shared parameters"
        In 1 : string = parameter
        In 2 : string = value
        Available parameters:
        PATH_OPENSTATION = folder path of the current .stn file
        FILE_OPENSTATION = file path of the current .stn file
        PATH_DESKTOP = folder path of the user's folder
        Other parameters can be added or modified by the user"""    
        self._check_connection()
        command = 'S_Param'
        self._send_line(command)
        self._send_line(str(param))
        self._send_line(str(value).replace('\n',' '))
        self._check_status()

    def ShowSequence(self, matrix):
        """Displays a sequence of joints
        In  1 : joint sequence as a 6xN matrix or instruction sequence as a 7xN matrix"""
        Item(self, 0).ShowSequence(matrix)

    def LaserTracker_Measure(self, estimate=[0,0,0], search=False):
        """Takes a laser tracker measurement with respect to the reference frame. If an estimate point is provided, the laser tracker will first move to those coordinates. If search is True, the tracker will search for a target.
        Returns the XYZ coordinates of target if it was found. Othewise it retuns None."""
        self._check_connection()
        command = 'MeasLT'
        self._send_line(command)
        self._send_xyz(estimate)
        self._send_int(1 if search else 0)
        xyz = self._rec_xyz()
        self._check_status()
        if xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2] < 0.0001:
            return None
        
        return xyz

    def Collision_Line(self, p1, p2, ref=eye(4)):
        """Checks the collision between a line and the station. The line is composed by 2 points.
        In  1 : p1 -> start point of the line
        In  2 : p2 -> end point of the line
        In  3 : pose (optional) -> reference of the 2 points
        Out 1 : collision -> True if there is a collision, False otherwise
        Out 2 : item -> Item collided
        Out 3 : point -> collision point (station reference)"""
        p1abs = ref*p1;
        p2abs = ref*p2;        
        self._check_connection()
        command = 'CollisionLine'
        self._send_line(command)
        self._send_xyz(p1abs)
        self._send_xyz(p2abs)
        itempicked = self._rec_item()
        xyz = self._rec_xyz()
        collision = itempicked.Valid()
        self._check_status()
        return collision, itempicked, xyz
        
    def setPoses(self, items, poses):
        """Sets the relative positions (poses) of a list of items with respect to their parent. For example, the position of an object/frame/target with respect to its parent.
        Use this function instead of setPose() for faster rendering.
        In 1 : List of items
        In 2 : List of poses"""
        if len(items) != len(poses):
            raise Exception('The number of items must match the number of poses')
        
        if len(items) == 0:
            return
            
        self._check_connection()
        command = 'S_Hlocals'
        self._send_line(command)
        self._send_int(len(items))
        for i in range(len(items)):
            self._send_item(items[i])
            self._send_pose(poses[i])
        self._check_status()
        
                
    def setPosesAbs(self, items, poses):
        """Sets the absolute positions (poses) of a list of items with respect to the station reference. For example, the position of an object/frame/target with respect to its parent.
        Use this function instead of setPose() for faster rendering.
        In 1 : List of items
        In 2 : List of poses"""
        if len(items) != len(poses):
            raise Exception('The number of items must match the number of poses')
        
        if len(items) == 0:
            return
            
        self._check_connection()
        command = 'S_Hlocal_AbsS'
        self._send_line(command)
        self._send_int(len(items))
        for i in range(len(items)):
            self._send_item(items[i])
            self._send_pose(poses[i])
        self._check_status()

        
    def Joints(self, robot_item_list):
        """Returns the current joints of a list of robots.
        In  1 : list of robot items
        Out 1 : list of robot joints (double x nDOF)"""
        self._check_connection()
        command = 'G_ThetasList'
        self._send_line(command)
        nrobs = len(robot_item_list)
        self._send_int(nrobs)
        joints_list = []
        for i in range(nrobs):
            self._send_item(robot_item_list[i])
            joints_i = self._rec_array()
            joints_list.append(joints_i)
        self._check_status()
        return joints_list

    def setJoints(self, robot_item_list, joints_list):
        """Sets the current robot joints for a list of robot items and a list of a set of joints.
        In  1 : list of robot items
        In  2 : list of robot joints (double x nDOF)"""
        nrobs = len(robot_item_list)
        if nrobs != len(joints_list):
            raise Exception('The size of the robot list does not match the size of the joints list')
            
        self._check_connection()
        command = 'S_ThetasList'
        self._send_line(command)
        self._send_int(nrobs)
        for i in range(nrobs):
            self._send_item(robot_item_list[i])
            self._send_array(joints_list[i])
            
        self._check_status()
        
    def CalibrateTCP(self, poses_xyzwpr, format=EULER_RX_RY_RZ, algorithm=CALIBRATE_TCP_BY_POINT):
        """Calibrate a TCP given a number of points following a certain algorithm.
        In  1 : poses_xyzwpr (matrix 6xN) : matrix of poses in a given format
        In  2 : format    (int)             : format
        In  3 : algorithm (int) - type of algorithm (by point, plane, ...)
        Out 1 : TCPxyz [x,y,z] - calculated TCP
        Out 2 : stats [mean, standard deviation, max] - error stats summary
        Out 3 : errors for each pose (array 1xN)"""
        self._check_connection()
        command = 'CalibTCP'
        self._send_line(command)
        self._send_matrix(poses_xyzwpr)
        self._send_int(format)
        self._send_int(algorithm)
        TCPxyz = self._rec_array()
        errorstats = self._rec_array()
        errors = self._rec_matrix()           
        self._check_status()
        errors = errors[:,1].tolist()
        return TCPxyz.tolist(), errorstats.tolist(), errors
        
        
    def ProgramStart(self, programname, folder='', postprocessor='', robot=None):
        """Defines the name of the program when the program is generated. It is also possible to specify the name of the post processor as well as the folder to save the program. 
        This method must be called before any program output is generated (before any robot movement or other instruction).
        
        :param progname: name of the program
        :type progname: str
        :param folder: folder to save the program, leave empty to use the default program folder
        :type folder: str
        :param postprocessor: name of the post processor (for a post processor in C:/RoboDK/Posts/Fanuc_post.py it is possible to provide "Fanuc_post.py" or simply "Fanuc_post")
        :type postprocessor: str
        :param robot: Robot to link
        :type robot: :class:`.Item`
        """
        self._check_connection()
        command = 'ProgramStart'
        self._send_line(command)
        self._send_line(programname)
        self._send_line(folder)
        self._send_line(postprocessor)        
        if robot is None:
            self._send_item(Item(None))
        else:
            self._send_item(robot)        
        errors = self._rec_int()
        self._check_status()
        return errors
        
    #------------------------------------------------------------------
    #----------------------- CAMERA VIEWS ----------------------------
    def Cam2D_Add(self, item_object, cam_params=""):
        """Adds a 2D camera view.
        In  1 : Parameters of the camera 
        Out 1 : camera handle pointer"""
        self._check_connection()
        command = 'Cam2D_Add'
        self._send_line(command)
        self._send_item(item_object)
        self._send_line(cam_params)
        cam_handle = self._rec_ptr()
        self._check_status()
        return cam_handle
        
    def Cam2D_Snapshot(self, file_save_img, cam_handle=0):
        """Returns the current joints of a list of robots.
        In  1 : Parameters of the camera 
        Out 1 :  1 if success, 0 otherwise"""
        self._check_connection()
        command = 'Cam2D_Snapshot'
        self._send_line(command)
        self._send_ptr(int(cam_handle))
        self._send_line(file_save_img)        
        success = self._rec_int()
        self._check_status()
        return success
        
    def Cam2D_Close(self, cam_handle=0):
        """Closes all camera windows or one specific camera if the camera handle is provided.
        In  1 : Camera handle (optional)
        Out 1 : 1 if success, 0 otherwise"""
        self._check_connection()
        if cam_handle == 0:
            command = 'Cam2D_CloseAll'
            self._send_line(command)
        else:
            command = 'Cam2D_Close'
            self._send_line(command)
            self._send_ptr(cam_handle)
        success = self._rec_int()
        self._check_status()
        return success
        
    def Cam2D_SetParams(self, params, cam_handle=0):
        """Set the parameters of the camera.
        In  1 : Parameters of the camera 
        Out 1 :  1 if success, 0 otherwise"""
        self._check_connection()
        command = 'Cam2D_SetParams'
        self._send_line(command)
        self._send_ptr(int(cam_handle))
        self._send_line(params)        
        success = self._rec_int()
        self._check_status()
        return success

    
class Item():
    """The Item class represents an item in RoboDK station. An item can be a robot, a frame, a tool, an object, a target, ... any item visible in the station tree.
    An item can also be seen as a node where other items can be attached to (child items).
    Every item has one parent item/node and can have one or more child items/nodes"""
    
    def __init__(self, link, ptr_item=0, itemtype=-1):
        self.item = ptr_item
        self.link = link # it is important to keep this as a reference and not a duplicate (otherwise it will establish a new connection at every call)
        self.type = itemtype

    def __repr__(self):
        if self.Valid():
            return ("RoboDK item (%i) of type %i" % (self.item, int(self.type)))
        else:
            return "RoboDK item (INVALID)"
            
    def __cmp__(self, item2):
        return self.item - item2.item
    
    def equals(self, item2):
        return self.item == item2.item
    
    def RDK(self):
        """Returns the RoboDK link Robolink()."""
        return self.link
        
    def RL(self):
        """Returns the RoboDK link Robolink() (old version)."""
        return self.link
    
    #"""Generic item calls"""
    def Type(self):
        """Returns an integer that represents the type of the item (robot, object, tool, frame, ...)
        Compare the returned value to ITEM_CASE_* variables"""
        self.link._check_connection()
        command = 'G_Item_Type'
        self.link._send_line(command)
        self.link._send_item(self)
        itemtype = self.link._rec_int()
        self.link._check_status()
        return itemtype
        
    def Copy(self):
        """Copy the item to the clipboard (same as Ctrl+C). Use together with Paste() to duplicate items."""
        self.link.Copy(self.item)
        
    def Paste(self):
        """Paste the item from the clipboard as a child of this item (same as Ctrl+V)
        Out 1: item -> new item pasted (created)"""
        return self.link.Paste(self.item)
        
    def AddFile(self, filename):
        """Adds an object as a child of this object"""
        return self.link.AddFile(filename, self.item)
        
    def Save(self, filename):
        """Save a station or object to a file"""
        self.link.Save(filename, self.item)
        
    def Collision(self, item_check):
        """Checks if this item is in a collision state with another item"""
        return self.link.Collision(self.item, item_check)
        
    def IsInside(self, object):
        """Returns True if the object is inside the provided object"""
        return self.link.IsInside(self.item, object)

    def AddGeometry(self, fromitem, pose):
        """Makes a copy of the geometry fromitem adding it at a given position (pose) relative to this item."""
        self.link._check_connection()
        command = 'CopyFaces'
        self.link._send_line(command)
        self.link._send_item(fromitem)
        self.link._send_item(self)
        self.link._send_pose(pose)        
        self.link._check_status()
        
    def Delete(self):
        """Deletes an item and its childs from the station.
        In  1 : item -> item to delete"""
        self.link._check_connection()
        command = 'Remove'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._check_status()
        self.item = 0

    def Valid(self):
        """Checks if the item is valid. An invalid item will be returned by an unsuccessful function call."""
        if self.item == 0: return False
        return True
    
    def setParent(self, parent):
        """Moves the item to another location item node "parent" (a different parent within the tree)
        In 1  : parent -> parent item to attach the item"""
        self.link._check_connection()
        command = 'S_Parent'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_item(parent)
        self.link._check_status()
        
    def setParentStatic(self, parent):
        """Moves the item to another location (parent) without changing the current position in the station
        In 1  : parent -> parent to attach the item"""
        self.link._check_connection()
        command = 'S_Parent_Static'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_item(parent)
        self.link._check_status()

    def AttachClosest(self):
        """Attaches the closest object to the provided tool (see also: Set_Parent_Static).
        Out  : item -> returns the item that was attached (item.Valid() is False if none found)"""
        self.link._check_connection()
        command = 'Attach_Closest'
        self.link._send_line(command)
        self.link._send_item(self)
        item_attached = self.link._rec_item()
        self.link._check_status()
        return item_attached

    def DetachClosest(self, parent=0):
        """Detaches the closest object attached to the tool (see also: setParentStatic).
        In 1 : parent item (optional) -> parent to leave the object."""
        self.link._check_connection()
        command = 'Detach_Closest'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_item(parent)
        item_detached = self.link._rec_item()
        self.link._check_status()
        return item_detached        

    def DetachAll(self, parent=0):
        """Detaches any object attached to a tool (see also: setParentStatic).
        In  1 : item (optional) -> parent to leave the objects"""
        self.link._check_connection()
        command = 'Detach_All'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_item(parent)
        self.link._check_status()

    def Parent(self):
        """Returns the parent item of the item.
        Out : parent -> parent of the item"""
        self.link._check_connection()
        command = 'G_Parent'
        self.link._send_line(command)
        self.link._send_item(self)
        parent = self.link._rec_item()
        self.link._check_status()
        return parent

    def Childs(self):
        """Returns a list of the item childs that are attached to the provided item.
        Out  : item x n -> list of child items"""
        self.link._check_connection()
        command = 'G_Childs'
        self.link._send_line(command)
        self.link._send_item(self)
        nitems = self.link._rec_int()
        itemlist = []
        for i in range(nitems):
            itemlist.append(self.link._rec_item())
        self.link._check_status()
        return itemlist

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def Visible(self):
        """Returns 1 if the item is visible, otherwise, returns 0.
        Out : int -> visible (1) or not visible (0)"""
        self.link._check_connection()
        command = 'G_Visible'
        self.link._send_line(command)
        self.link._send_item(self)
        visible = self.link._rec_int()
        self.link._check_status()
        return visible

    def setVisible(self, visible, visible_frame=None):
        """Sets the item visiblity status
        In 1 : int -> set visible (True or 1) or not visible (False or 0)
        In 2 : int (optional) -> set visible frame (True or 1) or not visible (False or 0)"""        
        if visible_frame is None: visible_frame = visible
        self.link._check_connection()
        command = 'S_Visible'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(visible)
        self.link._send_int(visible_frame)
        self.link._check_status()

    def Name(self):
        """Returns the name of an item. The name of the item is always displayed in the RoboDK station tree
        Out : name (string)"""
        self.link._check_connection()
        command = 'G_Name'
        self.link._send_line(command)
        self.link._send_item(self)
        name = self.link._rec_line()
        self.link._check_status()
        return name

    def setName(self, name):
        """Sets the name of an item.
        In 1 : name (string)"""
        self.link._check_connection()
        command = 'S_Name'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(name)
        self.link._check_status()
        
    def setValue(self, varname, value):
        """Sets a any property value to an item.
        In 1 : property name (string)
        In 2 : property value"""
        self.link._check_connection()
        if isinstance(value, Mat):
            command = 'S_Gen_Mat'
            self.link._send_line(command)
            self.link._send_item(self)
            self.link._send_line(varname)
            self.link._send_matrix(value)
        elif isinstance(value,str):
            command = 'S_Gen_Str'
            self.link._send_line(command)
            self.link._send_item(self)
            self.link._send_line(varname)
            self.link._send_line(value)
        else:
            raise Exception("Unsupported value type")
        self.link._check_status()
        
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def setPose(self, pose):
        """Sets the local position (pose) of an object, target or reference frame. For example, the position of an object/frame/target with respect to its parent.
        If a robot is provided, it will set the pose of the end efector.
        In 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'S_Hlocal'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_pose(pose)
        self.link._check_status()

    def Pose(self):
        """Returns the local position (pose) of an object, target or reference frame. For example, the position of an object, target or reference frame with respect to its parent.
        If a robot is provided, it will get the pose of the end efector.
        Out : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Hlocal'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose
        
    def setGeometryPose(self, pose):
        """Sets the position (pose) the object geometry with respect to its own reference frame. This procedure works for tools and objects.
        In 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'S_Hgeom'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_pose(pose)
        self.link._check_status()

    def GeometryPose(self):
        """Returns the position (pose) the object geometry with respect to its own reference frame. This procedure works for tools and objects.
        Out : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Hgeom'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose

    def setPoseAbs(self, pose):
        """Sets the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
        In  1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'S_Hlocal_Abs'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_pose(pose)
        self.link._check_status()

    def PoseAbs(self):
        """Returns the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Hlocal_Abs'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose

    def Recolor(self, tocolor, fromcolor=None, tolerance=None):
        """Changes the color of a robot/object/tool. A color must must in the format COLOR=[R,G,B,(A=1)] where all values range from 0 to 1.
        Alpha (A) defaults to 1 (100% opaque). Set A to 0 to make an object transparent.
        In  1 : color -> color to change to
        In  2 (optional): color -> filter by self color
        In  3 (optional): int -> optional tolerance to use if a color filter is used (defaults to 0.1)"""
        self.link._check_connection()
        if not fromcolor:
            fromcolor = [0,0,0,0]
            tolerance = 2
        elif not tolerance:
            tolerance= 0.1
        if not (isinstance(tolerance,int) or isinstance(tolerance,float)):
            raise Exception("tolerance must be a scalar")
            
        tocolor = self.link._check_color(tocolor)
        fromcolor = self.link._check_color(fromcolor)
        command = 'Recolor'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array([tolerance] + fromcolor + tocolor)
        self.link._check_status()
        
    def setColor(self, tocolor):
        """Set the color of an object, tool or robot. A color must must in the format COLOR=[R,G,B,(A=1)] where all values range from 0 to 1.
        In  1 : color -> color to change to"""
        self.link._check_connection()            
        tocolor = self.link._check_color(tocolor)
        command = 'S_Color'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array(tocolor)
        self.link._check_status()
        
    def Color(self):
        """Returns the color of an object, tool or robot (first color found). A color is in the format COLOR=[R,G,B,(A=1)] where all values range from 0 to 1."""
        self.link._check_connection()            
        command = 'G_Color'
        self.link._send_line(command)
        self.link._send_item(self)
        color = self.link._rec_array()
        self.link._check_status()
        return color.tolist()
    
    def Scale(self, scale):
        """Apply a scale to an object to make it bigger or smaller.
        the scale can be uniform (if scale is a float value) or per axis (if scale is a vector [scale_x, scale_y, scale_z]).
        In  1 : scale -> scale to apply"""
        self.link._check_connection()
        if isinstance(scale,float) or isinstance(scale,int):
            scale = [scale, scale, scale]
        elif len(scale) > 3:
            scale = scale[:3]
        elif len(scale) < 3:
            raise Exception("scale must be a single value or a 3-vector value")
        command = 'Scale'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array(scale)
        self.link._check_status()
        
    #"""Object specific calls"""
    def AddShape(self, triangle_points):
        """Adds a shape to the object provided some triangle coordinates. Triangles must be provided as a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3 because vertices must be stacked by groups of 3. Each group is a triangle."""
        return self.link.AddShape(triangle_points, self)
    
    def AddCurve(self, curve_points, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Adds a curve provided point coordinates. The provided points must be a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3
        In 2 (optional): add_to_ref -> If True, the curve will be added as part of the object in the RoboDK item tree
        In 3 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected."""
        return self.link.AddCurve(curve_points, self, add_to_ref, projection_type)        
    
    def AddPoints(self, points, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Adds a list of points to an object. The provided points must be a list of vertices. A vertex normal can be provided optionally.
        In 1  : matrix 3xN or 6xN -> N must be multiple of 3
        In 2 (optional): add_to_ref -> If True, the curve will be added as part of the object in the RoboDK item tree
        In 3 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected."""
        return self.link.AddPoints(points, self, add_to_ref, projection_type)        
        
    def ProjectPoints(self, points, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        """Projects a point to the object given its coordinates. The provided points must be a list of [XYZ] coordinates. Optionally, a vertex normal can be provided [XYZijk].
        In 1  : matrix 3xN or 6xN -> list of points to project
        In 2 (optional): projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.
        Out 1 : item -> projected points (empty matrix if failed)"""
        return self.link.ProjectPoints(points, self, projection_type)
   
    def setMillingParameters(self, ncfile='', part=0, params=''):
        """Adds a new machining project. Machining projects can also be used for 3D printing, curve following and point following.
        In  1 : string -> name of the project
        In  2 (optional): item -> robot that will be used
        Out 1 : item -> the new item created"""
        self.link._check_connection()
        command = 'S_MachiningParams'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(ncfile)
        self.link._send_item(part)
        self.link._send_line(params)
        newprog = self.link._rec_item()
        status = self.link._rec_int()/1000.0
        self.link._check_status()
        return newprog, status
        
    #"""Target item calls"""
    def setAsCartesianTarget(self):
        """Sets a target as a cartesian target. A cartesian target moves to cartesian coordinates."""
        self.link._check_connection()
        command = 'S_Target_As_RT'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._check_status()
    
    def setAsJointTarget(self):
        """Sets a target as a joint target. A joint target moves to a joints position without regarding the cartesian coordinates."""
        self.link._check_connection()
        command = 'S_Target_As_JT'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._check_status()
    
    #"""Robot item calls"""
    def Joints(self):
        """Returns the current joints of a robot or the joints of a target. If the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
        Out 1 : double x n -> joints matrix
        Example to convert a nx1 joint Matrix to a vector:
            joints = tr(robot.Joints())
            joints = joints.rows[0]"""
        self.link._check_connection()
        command = 'G_Thetas'
        self.link._send_line(command)
        self.link._send_item(self)
        joints = self.link._rec_array()
        self.link._check_status()
        return joints
        
    def JointPoses(self, joints = None):
        """Returns the positions of the joint links for a provided robot configuration (joints). If no joints are provided it will return the poses for the current robot position.
        Out 1 : 4x4 x n -> array of 4x4 homogeneous matrices. Index 0 is the base frame reference (it never moves when the joints move).
        """
        self.link._check_connection()
        command = 'G_LinkPoses'
        self.link._send_line(command)
        self.link._send_item(self)
        if joints is None:
            self.link._send_array([])
        else:
            self.link._send_array(joints)
            
        nlinks = self.link._rec_int()
        poses = []
        for i in range(nlinks):
            poses.append(self.link._rec_pose())
            
        self.link._check_status()
        return poses
    
    def JointsHome(self):
        """Returns the home joints of a robot. These joints can be manually set in the robot "Parameters" menu, then select "Set home position".
        Out 1 : double x n -> joints matrix"""
        self.link._check_connection()
        command = 'G_Home'
        self.link._send_line(command)
        self.link._send_item(self)
        joints = self.link._rec_array()
        self.link._check_status()
        return joints
        
    def ObjectLink(self, link_id=0):
        """Returns an item pointer to the link id (0 for the robot base, 1 for the first link, ...). This is useful if we want to show/hide certain links or change their geometry.
        In  1 : link_id -> joint id (0 for the base)
        Out 1 : item -> item object (pointer) to the robot link"""
        self.link._check_connection()
        command = 'G_LinkObjId'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(link_id)
        item = self.link._rec_item()
        self.link._check_status()
        return item
    
    def setJoints(self, joints):
        """Sets the current joints of a robot or the joints of a target. It the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
        In  1 : double x n -> joints"""
        self.link._check_connection()
        command = 'S_Thetas'
        self.link._send_line(command)
        self.link._send_array(joints)
        self.link._send_item(self)
        self.link._check_status()
        
    def JointLimits(self):
        """Returns the joint limits of a robot."""
        self.link._check_connection()
        command = 'G_RobLimits'
        self.link._send_line(command)
        self.link._send_item(self)
        lim_inf = self.link._rec_array()
        lim_sup = self.link._rec_array()        
        joints_type = self.link._rec_int()/1000.0
        self.link._check_status()
        return [lim_inf, lim_sup, joints_type]
    
    def setRobot(self, robot=None):
        """Sets the robot of a program or a target. You must set the robot linked to a program or a target every time you copy paste these objects.
        If the robot is not provided, the first available robot will be chosen automatically.
        In  1 : robot (optional) -> robot item"""
        if robot is None:
            robot = Item(self.link)
        self.link._check_connection()
        command = 'S_Robot'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_item(robot)
        self.link._check_status()

    def setFrame(self, frame):
        """Obsolete: use setPoseFrame instead"""
        self.setPoseFrame(frame)
        
    def setTool(self, tool):
        """Obsolete: use setPoseTool instead"""
        self.setPoseTool(tool)
        
    def setPoseFrame(self, frame):
        """Sets the reference frame of a robot (user frame). The frame can be either an item or a pose.
        If "frame" is an item, it links the robot to the frame item. If frame is a pose, it updates the linked pose of the robot frame.
        In  1 : item/pose -> frame item or 4x4 Matrix (pose of the reference frame)"""
        self.link._check_connection()
        if isinstance(frame,Item):
            command = 'S_Frame_ptr'
            self.link._send_line(command)
            self.link._send_item(frame)
        else:
            command = 'S_Frame'
            self.link._send_line(command)
            self.link._send_pose(frame)
        self.link._send_item(self)
        self.link._check_status()
    
    def setPoseTool(self, tool):
        """Sets the tool of a robot or a tool object (Tool Center Point, or TCP). The tool pose can be either an item or a 4x4 Matrix.
        If "tool" is an item, it links the robot to the tool item. If tool is a pose, it updates the current robot TCP.
        In  1 : item/pose -> tool item or 4x4 Matrix (pose of the tool frame)"""
        self.link._check_connection()
        if isinstance(tool,Item):
            command = 'S_Tool_ptr'
            self.link._send_line(command)
            self.link._send_item(tool)
        else:
            command = 'S_Tool'
            self.link._send_line(command)
            self.link._send_pose(tool)        
        self.link._send_item(self)
        self.link._check_status()
        
    def PoseTool(self):
        """Returns the tool pose of a robot or tool. If a robot is provided it will get the tool pose of the active tool held by the robot.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Tool'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose
        
    def PoseFrame(self):
        """Returns the reference frame of a robot. If a robot is provided it will get the tool pose of the active reference frame.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Frame'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose
    
    # Obsolete:    
    def setHtool(self, pose):
        """Note: this method is obsolete, use "setPoseTool" instead.
        Sets the tool pose of a tool item. If a robot is provided it will set the tool pose of the active tool held by the robot.
        In 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'S_Htool'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_pose(pose)
        self.link._check_status()

    # Obsolete:
    def Htool(self):
        """Note: this method is obsolete, use "PoseTool" instead.
        Returns the tool pose of an item. If a robot is provided it will get the tool pose of the active tool held by the robot.
        Out 1 : 4x4 homogeneous matrix (pose)"""
        self.link._check_connection()
        command = 'G_Htool'
        self.link._send_line(command)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose        
     
    def AddTool(self, tool_pose, tool_name = 'New TCP'):
        """Adds an empty tool to the robot provided the tool pose (4x4 Matrix) and the tool name.
        In  1: pose -> TCP as a 4x4 Matrix (pose of the tool frame)
        In  2: string -> New tool name
        Out 1: item -> New tool item"""
        self.link._check_connection()
        command = 'AddToolEmpty'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_pose(tool_pose)
        self.link._send_line(tool_name)
        newtool = self.link._rec_item()
        self.link._check_status()
        return newtool
    
    def SolveFK(self, joints):
        """Computes the forward kinematics of the robot for the provided joints.
        In  1 : double x n -> joints
        Out 1 : 4x4 matrix -> pose of the robot flange with respect to the robot base reference"""
        self.link._check_connection()
        command = 'G_FK'
        self.link._send_line(command)
        self.link._send_array(joints)
        self.link._send_item(self)
        pose = self.link._rec_pose()
        self.link._check_status()
        return pose
    
    def JointsConfig(self, joints):
        """Returns the robot configuration state for a set of robot joints.
        In  1 : double x n -> joints
        Out 1 : 3-array -> configuration status as [REAR, LOWERARM, FLIP]"""
        self.link._check_connection()
        command = 'G_Thetas_Config'
        self.link._send_line(command)
        self.link._send_array(joints)
        self.link._send_item(self)
        config = self.link._rec_array()
        self.link._check_status()
        return config
    
    def SolveIK(self, pose, joints_approx = None):
        """Computes the inverse kinematics for the specified robot and pose. The joints returned are the closest to the current robot configuration (see SolveIK_All()).
        Optionally, we can specify a preferred robot position using the second parameter joints_approx.
        In  1 : 4x4 matrix -> pose of the robot flange with respect to the robot base reference
        In  2 : double x n -> approximate joint solution
        Out 1 : double x n -> joints"""
        self.link._check_connection()
        if joints_approx is None:
            command = 'G_IK'
            self.link._send_line(command)
            self.link._send_pose(pose)
            self.link._send_item(self)
            joints = self.link._rec_array()
        else:
            command = 'G_IK_jnts'
            self.link._send_line(command)
            self.link._send_pose(pose)
            self.link._send_array(joints_approx)
            self.link._send_item(self)
            joints = self.link._rec_array()        
        self.link._check_status()
        return joints
    
    def SolveIK_All(self, pose):
        """Computes the inverse kinematics for the specified robot and pose. The function returns all available joint solutions as a 2D matrix.
        In  1 : 4x4 matrix -> pose of the robot TCP with respect to the robot reference frame
        Out 1 : double x n x m -> joint list (2D matrix)"""
        self.link._check_connection()
        command = 'G_IK_cmpl'
        self.link._send_line(command)
        self.link._send_pose(pose)
        self.link._send_item(self)
        joints_list = self.link._rec_matrix()
        self.link._check_status()
        return joints_list
        
    def FilterTarget(self, pose, joints_approx=None):
        """Filters a target to improve accuracy.
        In  1 : 4x4 matrix -> pose of the robot TCP with respect to the robot reference frame
        In  2 : double x n -> approximate joints to define preferred configuration
        Out 1 : double x n x m -> joint list (2D matrix)"""
        self.link._check_connection()
        command = 'FilterTarget'
        self.link._send_line(command)
        self.link._send_pose(pose)
        if joints_approx is None:
            joints_approx = [0,0,0,0,0,0]
        self.link._send_array(joints_approx)
        self.link._send_item(self)
        pose_filtered = self.link._rec_pose()
        joints_filtered = self.link._rec_array()
        self.link._check_status()
        return [pose_filtered, joints_filtered]
    
    def Connect(self, robot_ip = ''):
        """Connect to a real robot using the robot driver.
        In  1 : robot_ip (string) -> IP of the robot to connect. Leave blank to use the IP selected in the connection panel of the robot.
        Out 1 : status -> 1 if connection succeeded 0 if it failed"""
        self.link._check_connection()
        command = 'Connect'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(robot_ip)        
        status = self.link._rec_int()
        self.link._check_status()
        return status
        
    def ConnectSafe(self, robot_ip = '', max_attempts=5, wait_connection=4):
        """Connect to a real robot and wait for a connection to succeed.
        In  1 : robot_ip (string) -> IP of the robot to connect. Leave blank to use the IP selected in the connection panel of the robot.
        Out 1 : status -> 1 if connection succeeded 0 if it failed"""    
        trycount = 0
        refresh_rate = 0.5
        self.Connect()
        tic()
        timer1 = toc()
        pause(refresh_rate)
        while True:
            con_status, status_msg = self.ConnectedState()
            print(status_msg)
            if con_status == ROBOTCOM_READY:
                print(status_msg)
                break
            elif con_status == ROBOTCOM_DISCONNECTED:
                print('Trying to reconnect...')
                self.Connect()
            if toc() - timer1 > wait_connection:
                timer1 = toc()
                self.Disconnect()
                trycount = trycount + 1
                if trycount >= max_attempts:
                    print('Failed to connect: Timed out')
                    break
                print('Retrying connection...')
            pause(refresh_rate)
        return con_status

        
    def ConnectionParams(self):
        """Retrieve robot connection parameters
        Out 1 : string -> robot IP
        Out 2 : int    -> port connection
        Out 3 : string -> remote path
        Out 4 : string -> FTP user name
        Out 5 : string -> FTP password"""
        self.link._check_connection()
        command = 'ConnectParams'
        self.link._send_line(command)
        self.link._send_item(self)
        robot_ip = self.link._rec_line()
        port = self.link._rec_int()
        remote_path = self.link._rec_line()
        ftp_user = self.link._rec_line()
        ftp_pass = self.link._rec_line()
        self.link._check_status()
        return [robot_ip, port, remote_path, ftp_user, ftp_pass]
        
    def setConnectionParams(self, robot_ip, port, remote_path, ftp_user, ftp_pass):
        """Retrieve robot connection parameters
        In 1 : string -> robot IP
        In 2 : int    -> port connection
        In 3 : string -> remote path
        In 4 : string -> FTP user name
        In 5 : string -> FTP password"""
        self.link._check_connection()
        command = 'setConnectParams'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(robot_ip)
        self.link._send_int(port)
        self.link._send_line(remote_path)
        self.link._send_line(ftp_user)
        self.link._send_line(ftp_pass)
        self.link._send_line(robot_ip)
        self.link._check_status()
        return [robot_ip, port, remote_path, ftp_user, ftp_pass]
        
    def ConnectedState(self):
        """Check connection status with a real robobt
        Out 1 : status code -> (int) ROBOTCOM_READY if the robot is ready to move, otherwise, status message will provide more information about the issue
        Out 2 : status message -> Message description of the robot status"""
        self.link._check_connection()
        command = 'ConnectedState'
        self.link._send_line(command)
        self.link._send_item(self)
        robotcom_status = self.link._rec_int()
        status_msg = self.link._rec_line()        
        self.link._check_status()
        return robotcom_status, status_msg
        
    def Disconnect(self):
        """Disconnect from a real robot (when the robot driver is used)
        Out 1 : status -> 1 if disconnected successfully, 0 if it failed. It can fail if it was previously disconnected manually for example."""
        self.link._check_connection()
        command = 'Disconnect'
        self.link._send_line(command)
        self.link._send_item(self)
        status = self.link._rec_int()
        self.link._check_status()
        return status
    
    def MoveJ(self, target, blocking=True):
        """Moves a robot to a specific target ("Move Joint" mode). This function blocks until the robot finishes its movements.
        In  1 : joints/pose/target -> target to move to. It can be the robot joints (Nx1 or 1xN), the pose (4x4) or a target (item pointer)
        In  2 (optional): blocking -> True if we want the instruction to wait until the robot finished the movement (default=True)"""
        self.link._moveX(target, self, 1, blocking)
    
    def MoveL(self, target, blocking=True):
        """Moves a robot to a specific target ("Move Linear" mode). This function waits (blocks) until the robot finishes its movements.
        In  1 : joints/pose/target -> target to move to. It can be the robot joints (Nx1 or 1xN), the pose (4x4) or a target (item pointer)
        In  2 (optional): blocking -> True if we want the instruction to wait until the robot finished the movement (default=True)"""
        self.link._moveX(target, self, 2, blocking)
        
    def MoveC(self, target1, target2, blocking=True):
        """Moves a robot to a specific target ("Move Circular" mode). This function waits (blocks) until the robot finishes its movements.
        In  1 : joints/pose/target -> target to move to. It can be the robot joints (Nx1 or 1xN), the pose (4x4) or a target (item pointer)
        In  2 (optional): blocking -> True if we want the instruction to wait until the robot finished the movement (default=True)"""
        self.link.MoveC(target1, target2, self, blocking)
    
    def MoveJ_Collision(self, j1, j2, minstep_deg=-1):
        """Checks if a joint movement is free of collision.
        In  1 : joints -> start joints
        In  2 : joints -> destination joints
        In  3 (optional): maximum joint step in degrees
        Out : collision : returns 0 if the movement is free of collision. Otherwise it returns the number of pairs of objects that collided if there was a collision."""
        self.link._check_connection()
        command = 'CollisionMove'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array(j1)
        self.link._send_array(j2)        
        self.link._send_int(minstep_deg*1000)
        collision = self.link._rec_int()
        self.link._check_status()
        return collision
    
    def MoveL_Collision(self, j1, pose, minstep_deg=-1):
        """Checks if a joint movement is free of collision.
        In  1 : joints -> start joints
        In  2 : pose -> destination pose
        In  3 (optional): maximum joint step in degrees
        Out : collision : returns 0 if the movement is free of collision. Otherwise it returns the number of pairs of objects that collided if there was a collision.
             if the robot can not reach the target pose it returns -2. If the robot can reach the target but it can not make a linear movement it returns -1"""
        self.link._check_connection()
        command = 'CollisionMoveL'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array(j1)
        self.link._send_pose(pose)        
        self.link._send_int(minstep_deg*1000)
        collision = self.link._rec_int()
        self.link._check_status()
        return collision
    
    def setSpeed(self, speed_linear, speed_joints=-1, accel_linear=-1, accel_joints=-1):
        """Sets the linear speed of a robot. Additional arguments can be provided to set linear acceleration or joint speed and acceleration.
        In  1 : linear speed -> speed in mm/s (-1 = no change)
        In  2 : joint speed (optional) -> acceleration in mm/s2 (-1 = no change)
        In  3 : linear acceleration (optional) -> acceleration in mm/s2 (-1 = no change)
        In  4 : joint acceleration (optional) -> acceleration in deg/s2 (-1 = no change)"""
        self.link._check_connection()
        command = 'S_Speed4'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array([float(speed_linear), float(speed_joints), float(accel_linear), float(accel_joints)])
        self.link._check_status()
    
    def setAcceleration(self, accel_linear):
        """Sets the linear acceleration of a robot in mm/s2
        In  1 : angular speed -> acceleration in mm/s2"""
        self.setSpeed(-1,accel_linear,-1,-1)
    
    def setSpeedJoints(self, speed_joints):
        """Sets the joint speed of a robot in deg/s for rotary joints and mm/s for linear joints
        In  1 : joint speed -> speed in deg/s for rotary joints and mm/s for linear joints"""
        self.setSpeed(-1,-1,speed_joints,-1)
    
    def setAccelerationJoints(self, accel_joints):
        """Sets the joint acceleration of a robot
        In  1 : joint acceleration -> acceleration in deg/s2 for rotary joints and mm/s2 for linear joints"""
        self.setSpeed(-1,-1,-1,accel_joints)   
    
    def setZoneData(self, zonedata):
        """Sets the robot zone data value.
        In  1 : zonedata value (int) (robot dependent, set to -1 for fine movements)"""
        self.link._check_connection()
        command = 'S_ZoneData'
        self.link._send_line(command)
        self.link._send_int(zonedata*1000);
        self.link._send_item(self)
        self.link._check_status()
    
    def ShowSequence(self, matrix):
        """Displays a sequence of joints
        In  1 : joint sequence as a 6xN matrix or instruction sequence as a 7xN matrix"""
        self.link._check_connection()
        command = 'Show_Seq'
        self.link._send_line(command)
        self.link._send_matrix(matrix);
        self.link._send_item(self)
        self.link._check_status()
    
    def Busy(self):
        """Checks if a robot or program is currently running (busy or moving).
        Out 1 : int -> busy status (1=moving, 0=stopped)"""
        self.link._check_connection()
        command = 'IsBusy'
        self.link._send_line(command)
        self.link._send_item(self)
        busy = self.link._rec_int()
        self.link._check_status()
        return busy
            
    def Stop(self):
        """Stops a program or a robot"""
        self.link._check_connection()
        command = 'Stop'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._check_status()
    
    def WaitMove(self, timeout=300):
        """Waits (blocks) until the robot finishes its movement.
        In  1 (optional): timeout -> Max time to wait for robot to finish its movement (in seconds)"""
        self.link._check_connection()
        command = 'WaitMove'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._check_status()
        self.link.COM.settimeout(timeout)
        self.link._check_status()#will wait here
        self.link.COM.settimeout(self.link.TIMEOUT)
        #busy = self.link.Is_Busy(self.item)
        #while busy:
        #    busy = self.link.Is_Busy(self.item)        
        
    def WaitFinished(self):
        """Wait until the program finishes."""
        while self.Busy():
            pause(0.05)
    
    def ProgramStart(self, programname, folder='', postprocessor=''):
        """Defines the name of the program when the program is generated. It is also possible to specify the name of the post processor as well as the folder to save the program. 
        This method must be called before any program output is generated (before any robot movement or other instruction).
        
        :param progname: name of the program
        :type progname: str
        :param folder: folder to save the program, leave empty to use the default program folder
        :type folder: str
        :param postprocessor: name of the post processor (for a post processor in C:/RoboDK/Posts/Fanuc_post.py it is possible to provide "Fanuc_post.py" or simply "Fanuc_post")
        :type postprocessor: str      
        """
        return self.link.ProgramStart(programname, folder, postprocessor, self)    
    
    def setAccuracyActive(self, accurate = 1):
        """Sets the accuracy of the robot active or inactive.
        In  1 : accurate -> 1 if True 0 if False"""
        self.link._check_connection()
        command = 'S_AbsAccOn'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(accurate)
        self.link._check_status()
    
    def FilterProgram(self, filestr):
        """Filters a program file to improve accuracy for a specific robot. The robot must have been previously calibrated.
        In  1 : string -> File path of the program.
        Out 1 : int -> status : 0 if the filter succeeded, -1 if there are conversion problems. Filter program manually to get a summary of the conversion."""
        self.link._check_connection()
        command = 'FilterProg'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(filestr)
        filter_status = self.link._rec_int()
        self.link._check_status()
        return filter_status
    
    #"""Program item calls"""    
    def MakeProgram(self, filestr):
        """Saves a program to a file.
        In  1 : string -> File path of the program
        Out 1 : int -> number of instructions that can be executed"""
        self.link._check_connection()
        command = 'MakeProg'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(filestr)
        prog_status = self.link._rec_int()
        prog_log_str = self.link._rec_line()
        self.link._check_status()
        if prog_status > 1:
            success = True
        else:
            success = False
        return [success, prog_log_str]
    
    def setRunType(self, program_run_type):
        """Sets if the program will be run in simulation mode or on the real robot.
        Use: "PROGRAM_RUN_ON_SIMULATOR" to set the program to run on the simulator only or "PROGRAM_RUN_ON_ROBOT" to force the program to run on the robot"""
        self.link._check_connection()
        command = 'S_ProgRunType'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(program_run_type)
        self.link._check_status()
        
    def RunCode(self, prog_parameters=None):
        """Runs a program. It returns the number of instructions that can be executed successfully (a quick program check is performed before the program starts)
        Program parameters can be provided for Python calls.
        This is a non-blocking call. Use IsBusy() to check if the program execution finished.
        Notes: if setRunMode(RUNMODE_SIMULATE) is used  -> the program will be simulated (default run mode)
               if setRunMode(RUNMODE_RUN_ROBOT) is used ->the program will run on the robot (default when RUNMODE_RUN_ROBOT is used)
               if setRunMode(RUNMODE_RUN_ROBOT) is used together with program.setRunType(PROGRAM_RUN_ON_ROBOT) -> the program will run sequentially on the robot the same way as if we right clicked the program and selected "Run on robot" in the RoboDK GUI
        Out 1 : int -> number of instructions that can be executed"""
        self.link._check_connection()
        if type(prog_parameters) == list:
            command = 'RunProgParam'
            self.link._send_line(command)
            self.link._send_item(self)
            parameters = '<br>'.join(str(param_i) for param_i in prog_parameters)
            self.link._send_line(parameters)
        else:
            command = 'RunProg'
            self.link._send_line(command)
            self.link._send_item(self)
        prog_status = self.link._rec_int()
        self.link._check_status()
        return prog_status
        
    def RunCodeCustom(self, code, run_type=INSTRUCTION_CALL_PROGRAM):
        """Adds a program call, code, message or comment inside a program.
        In  1 : code -> string of the code or program to run
        In  2 : run_type -> INSTRUCTION_* variable to specify if the code is a progra
        Out 1 : this function always returns 0"""
        self.link._check_connection()
        command = 'RunCode2'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(code.replace('\r\n','<<br>>').replace('\n','<<br>>'))
        self.link._send_int(run_type)        
        prog_status = self.link._rec_int()
        self.link._check_status()
        return prog_status
        
    def Pause(self, time_ms = -1):
        """Generates a pause instruction for a robot or a program when generating code. Do not provide a pause time if you want the robot to stop and let the user resume the program anytime.
        In 1 : int -> time in miliseconds"""
        self.link._check_connection()
        command = 'RunPause'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(time_ms*1000.0)
        self.link._check_status()
        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value. This can also be used to set any variables to a desired value.
        In 1 : io_var -> digital output (string or number)
        In 2 : io_value -> value (string or number)"""
        self.link._check_connection()
        command = 'setDO'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(str(io_var))
        self.link._send_line(str(io_value))
        self.link._check_status()
    
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_id to attain a given value io_value. Optionally, a timeout can be provided.
        In 1 : io_var -> digital output (string or number)
        In 2 : io_value -> value (string or number)
        In 3 : int (optional) -> timeout in miliseconds"""
        self.link._check_connection()
        command = 'waitDI'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_line(str(io_var))
        self.link._send_line(str(io_value))
        self.link._send_int(timeout_ms*1000)
        self.link._check_status()
    
    def addMoveJ(self, itemtarget):
        """Adds a new robot move joint instruction to a program.
        In  1 : item -> target to move to"""
        self.link._check_connection()
        command = 'Add_INSMOVE'
        self.link._send_line(command)
        self.link._send_item(itemtarget)
        self.link._send_item(self)
        self.link._send_int(1)
        self.link._check_status()
    
    def addMoveL(self, itemtarget):
        """Adds a new robot move linear instruction to a program.
        In  1 : item -> target to move to"""
        self.link._check_connection()
        command = 'Add_INSMOVE'
        self.link._send_line(command)
        self.link._send_item(itemtarget)
        self.link._send_item(self)
        self.link._send_int(2)
        self.link._check_status()
        
    def ShowInstructions(self, show=True):
        """Show instructions inside a program in RoboDK tree"""
        self.link._check_connection()
        command = 'Prog_ShowIns'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(1 if show else 0)        
        self.link._check_status()
        
    def InstructionCount(self):
        """Returns the number of instructions of a program."""
        self.link._check_connection()
        command = 'Prog_Nins'
        self.link._send_line(command)
        self.link._send_item(self)
        nins = self.link._rec_int()
        self.link._check_status()
        return nins
    
    def Instruction(self, ins_id=-1):
        """Returns the current program instruction or the instruction by id (if provided)."""
        self.link._check_connection()
        command = 'Prog_GIns'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(ins_id)
        name = self.link._rec_line()
        instype = self.link._rec_int()
        movetype = None
        isjointtarget = None
        target = None
        joints = None
        if instype == INS_TYPE_MOVE:
            movetype = self.link._rec_int()
            isjointtarget = self.link._rec_int()
            target = self.link._rec_pose()
            joints = self.link._rec_array()
        self.link._check_status()
        return [name, instype, movetype, isjointtarget, target, joints]
    
    def setInstruction(self, ins_id, name, instype, movetype, isjointtarget, target, joints):
        """Sets the program instruction at position id."""
        self.link._check_connection()
        command = 'Prog_SIns'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_int(ins_id)
        self.link._send_line(name)
        self.link._send_int(instype)
        if instype == INS_TYPE_MOVE:
            self.link._send_int(movetype)
            self.link._send_int(isjointtarget)
            self.link._send_pose(target)
            self.link._send_array(joints)
        self.link._check_status()
      
    def InstructionList(self):
        """Returns the list of program instructions as an MxN matrix, where N is the number of instructions and M equals to 1 plus the number of robot axes.
        Out 1: Returns the matrix
        Out 2: Returns 0 if success"""
        self.link._check_connection()
        command = 'G_ProgInsList'
        self.link._send_line(command)
        self.link._send_item(self)
        insmat = self.link._rec_matrix()
        errors = self.link._rec_int()
        self.link._check_status()
        return insmat, errors
          
    def InstructionListJoints(self, mm_step=10, deg_step=5, save_to_file = None):
        """Returns a list of joints an MxN matrix, where M is the number of robot axes plus 4 columns. Linear moves are rounded according to the smoothing parameter set inside the program.
        In  1: mm_step -> Maximum step in millimeters for linear movements (millimeters)
        In  2: joint_step -> Maximum step for joint movements (degrees)
        In  3: save_to_file (optional) -> Provide a file name to directly save the output to a file. If the file name is not provided it will return the matrix. If step values are very small, the returned matrix can be very large.
        Out 1: Returns a human readable error message (if any)
        Out 2: Returns the list of joints as [J1, J2, ..., Jn, ERROR, MM_STEP, DEG_STEP, MOVE_ID] or the file name if a file path is provided to save the result
        Out 3: Returns 0 if success, negative values"""
        self.link._check_connection()
        command = 'G_ProgJointList'
        self.link._send_line(command)
        self.link._send_item(self)
        self.link._send_array([mm_step, deg_step])
        joint_list = save_to_file   
        if save_to_file is None:
            self.link._send_line('')
            joint_list = self.link._rec_matrix()
        else:
            self.link._send_line(save_to_file)
        error_code = self.link._rec_int()
        error_msg = self.link._rec_line()
        self.link._check_status()
        return error_msg, joint_list, error_code

