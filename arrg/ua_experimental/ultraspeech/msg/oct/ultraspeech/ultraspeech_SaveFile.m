% Auto-generated.  Do not edit!

% msg = ultraspeech_SaveFile()
%
% SaveFile message type, fields include:
%   roslib_Header header
% string filepath

% //! \htmlinclude SaveFile.msg.html
function msg = ultraspeech_SaveFile()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/jeff/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.filepath = '';
msg.md5sum_ = @ultraspeech_SaveFile___md5sum;
msg.type_ = @ultraspeech_SaveFile___type;
msg.serializationLength_ = @ultraspeech_SaveFile___serializationLength;
msg.serialize_ = @ultraspeech_SaveFile___serialize;
msg.deserialize_ = @ultraspeech_SaveFile___deserialize;
msg.message_definition_ = @ultraspeech_SaveFile___message_definition;

function x = ultraspeech_SaveFile___md5sum()
x = '02eadefcd88df0f76162b21327c4bd37';

function x = ultraspeech_SaveFile___message_definition()
x = [    'Header header\n' ...
    'string filepath\n' ...
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

function x = ultraspeech_SaveFile___type()
x = 'ultraspeech/SaveFile';

function l__ = ultraspeech_SaveFile___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 4 + numel(msg.filepath);

function dat__ = ultraspeech_SaveFile___serialize(msg__, seq__, fid__)
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
fwrite(fid__, numel(msg__.filepath), 'uint32');
fwrite(fid__, msg__.filepath, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = ultraspeech_SaveFile___deserialize(dat__, fid__)
msg__ = ultraspeech_SaveFile();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.filepath = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = ultraspeech_SaveFile___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

