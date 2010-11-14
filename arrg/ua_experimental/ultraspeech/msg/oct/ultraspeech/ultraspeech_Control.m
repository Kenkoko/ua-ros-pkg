% Auto-generated.  Do not edit!

% msg = ultraspeech_Control()
%
% Control message type, fields include:
%   roslib_Header header
% int8 run
% string directory

% //! \htmlinclude Control.msg.html
function msg = ultraspeech_Control()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/robotlab/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.run = int8(0);
msg.directory = '';
msg.md5sum_ = @ultraspeech_Control___md5sum;
msg.type_ = @ultraspeech_Control___type;
msg.serializationLength_ = @ultraspeech_Control___serializationLength;
msg.serialize_ = @ultraspeech_Control___serialize;
msg.deserialize_ = @ultraspeech_Control___deserialize;
msg.message_definition_ = @ultraspeech_Control___message_definition;

function x = ultraspeech_Control___md5sum()
x = '122bda80d342152b54964353521a1ca8';

function x = ultraspeech_Control___message_definition()
x = [    'Header header\n' ...
    'int8 run\n' ...
    'string directory\n' ...
    '\n' ...
    '================================================================================\n' ...
    'MSG: roslib/Header\n' ...
    '# Standard metadata for higher-level stamped data types.\n' ...
    '# This is generally used to communicate timestamped data \n' ...
    '# in a particular coordinate frame.\n' ...
    '# \n' ...
    '# sequence ID: consecutively increasing ID \n' ...
    'uint32 seq\n' ...
    '#Two-integer timestamp that is expressed as:\n' ...
    '# * stamp.secs: seconds (stamp_secs) since epoch\n' ...
    '# * stamp.nsecs: nanoseconds since stamp_secs\n' ...
    '# time-handling sugar is provided by the client library\n' ...
    'time stamp\n' ...
    '#Frame this data is associated with\n' ...
    '# 0: no frame\n' ...
    '# 1: global frame\n' ...
    'string frame_id\n' ...
    '\n' ...
    '\n' ...
];

function x = ultraspeech_Control___type()
x = 'ultraspeech/Control';

function l__ = ultraspeech_Control___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 1 ...
    + 4 + numel(msg.directory);

function dat__ = ultraspeech_Control___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
if (msg__.header.seq == 0)
    msg__.header.seq = seq__;
end
if (msg__.header.stamp.sec == 0 && msg__.header.stamp.nsec == 0)
    msg__.header.stamp = rosoct_time_now();
end
msg__.header.serialize_(msg__.header, seq__, fid__);
c__ = c__ + fwrite(fid__, msg__.run, 'int8');
fwrite(fid__, numel(msg__.directory), 'uint32');
fwrite(fid__, msg__.directory, 'uint8');
if( c__ ~= 1 )
    error('some members of msg ultraspeech:Control are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = ultraspeech_Control___deserialize(dat__, fid__)
msg__ = ultraspeech_Control();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
msg__.run = fread(fid__,1,'int8=>int8');
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.directory = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = ultraspeech_Control___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

