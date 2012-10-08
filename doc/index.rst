.. _ecto_openni:

ecto_openni
===========

OpenNI cells for ecto.  These should be dependent on only ecto and
openni, allowing packages like PCL and opencv to provide converters to
the data types they're comfortable with.

On recent Linux distributions, you might need to let OpenNI deal with
the Kinect and not the kernel. You need to remove the module:
::

    rmmod gspca_kinect

You can also blacklist it:
::

    echo “blacklist gspca_kinect” > /etc/modprobe.d/blacklist-psengine.conf

.. toctree::
   :maxdepth: 1

   cells
   examples

