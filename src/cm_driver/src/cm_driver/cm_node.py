#!/usr/bin/env python3
import rospy
from threading import Thread


class CMNode(Thread):

    """
    A ROS Node wrapper
    """

    def __init__(self, name):

        """
        Parameters
        ----------
        @param name : str
            The name of the ROS node
        """

        super(CMNode, self).__init__()
        self.name = name
        rospy.init_node(self.name)
        self.__running = True
        rospy.on_shutdown(self.__on_ros_shutdown)

    def __on_ros_shutdown(self):

        """
        Callback function called whenever rospy.spin() stops
        """

        rospy.loginfo('Node ' + self.name + ' is stopping.')
        self.on_ros_shutdown()
        self.__running = False
        if self.is_alive():
            self.join()
        rospy.loginfo('Node ' + self.name + ' is stopped.')

    def on_ros_shutdown(self):

        """
        User defined callback function called whenever rospy.spin() stops
        """

        pass

    def run(self):

        """
        This is a virtual method that corresponds to the code of the Node that runs continuously
        It should have a while loop calling the self.is_running() function.
        """

        raise NotImplementedError

    def is_running(self):

        """
        Returns
        -------
        @return __running : bool
            Whether the thread is supposed to be running
        """

        return self.__running
