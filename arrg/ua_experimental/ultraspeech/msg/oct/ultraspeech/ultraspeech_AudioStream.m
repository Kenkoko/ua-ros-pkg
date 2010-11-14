% Auto-generated.  Do not edit!

% msg = ultraspeech_AudioStream()
%
% AudioStream message type, fields include:
%   roslib_Header header
% uint32 which_channel
% single{} samples
% uint32 num_channels
% uint32 sample_rate

% //! \htmlinclude AudioStream.msg.html
function msg = ultraspeech_AudioStream()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/robotlab/ros/ros/core/roslib/msg/oct/roslib');
end


msg = [];
msg.header = roslib_Header();
msg.which_channel = uint32(0);
msg.samples = [];
msg.num_channels = uint32(0);
msg.sample_rate = uint32(0);
msg.md5sum_ = @ultraspeech_AudioStream___md5sum;
msg.type_ = @ultraspeech_AudioStream___type;
msg.serializationLength_ = @ultraspeech_AudioStream___serializationLength;
msg.serialize_ = @ultraspeech_AudioStream___serialize;
msg.deserialize_ = @ultraspeech_AudioStream___deserialize;
msg.message_definition_ = @ultraspeech_AudioStream___message_definition;

function x = ultraspeech_AudioStream___md5sum()
x = 'e75f13067c4581e6d6ac1ce905ecf22c';

function x = ultraspeech_AudioStream___message_definition()
x = [    'Header header\n' ...
    'uint32 which_channel\n' ...
    'float32[] samples\n' ...
    'uint32 num_channels\n' ...
    'uint32 sample_rate\n' ...
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

function x = ultraspeech_AudioStream___type()
x = 'ultraspeech/AudioStream';

function l__ = ultraspeech_AudioStream___serializationLength(msg)
l__ =  ...
    + msg.header.serializationLength_(msg.header) ...
    + 4 ...
    + 4 + numel(msg.samples) * (4) ...
    + 4 ...
    + 4;

function dat__ = ultraspeech_AudioStream___serialize(msg__, seq__, fid__)
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
c__ = c__ + fwrite(fid__, msg__.which_channel, 'uint32');
fwrite(fid__, numel(msg__.samples), 'uint32');
fwrite(fid__, msg__.samples(:), 'single');
c__ = c__ + fwrite(fid__, msg__.num_channels, 'uint32');
c__ = c__ + fwrite(fid__, msg__.sample_rate, 'uint32');
if( c__ ~= 3 )
    error('some members of msg ultraspeech:AudioStream are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = ultraspeech_AudioStream___deserialize(dat__, fid__)
msg__ = ultraspeech_AudioStream();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.header = roslib_Header();
msg__.header = msg__.header.deserialize_(msg__.header, fid__);
msg__.which_channel = fread(fid__,1,'uint32=>uint32');
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.samples = fread(fid__, size__, 'single');
msg__.num_channels = fread(fid__,1,'uint32=>uint32');
msg__.sample_rate = fread(fid__,1,'uint32=>uint32');
if( file_created__ )
    fclose(fid__);
end
function l__ = ultraspeech_AudioStream___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

