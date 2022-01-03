=================================================
ROS / ROS2 VIM Auto-Completion With YouCompleteMe
=================================================

* `Install YouCompleteMe <install-ycm_>`_

  * `Ubuntu <install-ycm-ubuntu_>`_

  * Windows

* Setup ROS Workspace

* Other tools

  * tabnine

  * colcon-ed & colcon-cd

  * ros_src_tools

* Contributing

.. _install-ycm:

---------------------
Install YouCompleteMe
---------------------

I recommend installing YCM using `Vundle <https://github.com/VundleVim/Vundle.vim>`_.
However, the installation cannot be completed through a simple ``:PluginInstall`` in vim.

If you **already has Vundle installed**, skip ahead to the `Compile YCM section for Ubuntu` or `Windows`

.. _install-ycm-ubuntu:

Ubuntu
======

Install Vundle
--------------

#. **Requirement**

   Make sure you have ``vim`` and ``git`` installed.

#. **Setup** `Vundle <https://github.com/VundleVim/Vundle.vim>`_

  Simply clone the repository into the ``~/.vim/bundle`` directory.

  .. code:: bash

     git clone https://github.com/VundleVim/Vundle.vim.git \
                ~/.vim/bundle/Vundle.vim

#. **Enable Vundle**

   Put the following at the top of your ``~/.vimrc`` file.
   If this file does not exist, create a new empty file under the same name with the following content.

   .. code:: vim
      :linenos:

      set nocompatible              " be iMproved, required
      filetype off                  " required

      " set the runtime path to include Vundle and initialize
      set rtp+=~/.vim/bundle/Vundle.vim
      call vundle#begin()
      " alternatively, pass a path where Vundle should install plugins
      "call vundle#begin('~/some/path/here')

      " let Vundle manage Vundle, required
      Plugin 'VundleVim/Vundle.vim'

      " All of your Plugins must be added before the following line
      call vundle#end()            " required
      filetype plugin indent on    " required
      " To ignore plugin indent changes, instead use:
      "filetype plugin on
      "
      " Brief help
      " :PluginList       - lists configured plugins
      " :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
      " :PluginSearch foo - searches for foo; append `!` to refresh local cache
      " :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
      "
      " see :h vundle for more details or wiki for FAQ
      " Put your non-Plugin stuff after this line
