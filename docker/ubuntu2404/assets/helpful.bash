# Project: ENPM662-Project1-Group1
# Description: Snippet for .bashrc

alias df='df -h'
alias grep='grep --color=auto'
alias ll='ls -lah --color=auto'
alias ls='ls --color=auto'
alias vi='vim'
export LS_COLORS="$LS_COLORS:ow=35"
export LANG=en_US.UTF-8
export COLCON_DEFAULTS_FILE=~/.config/colcon/colcon.yaml

#yellow prompt
PS1='\[\033[01;33m\]\u@\h:\w\[\033[00m\]\$ '

