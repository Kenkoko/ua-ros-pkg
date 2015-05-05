# IF SOMETHING DOES NOT WORK #

... on the machines in CS930, **REBOOT** the machine (and/or try a different one), before panicking. This is due to a nasty bug in gazebo.

# When you are done with a program (especially gazebo) #

... hit Control-C in the terminal where you launched the program. You may have to wait a minute or do this multiple times to close the program. You want to see the prompt again before you close the terminal and log out, because of the bug above.

# Syntax highlighting for .launch files (in gedit) #

  1. Open the language configuration file as root (It's really just an XML file):
```
sudo gedit /usr/share/gtksourceview-2.0/language-specs/xml.lang
```
  1. Add "`*`.launch;`*`.world" to the list of blob types near the beginning of the XML. That's it!