#ifndef H9MSG_H
#define H9MSG_H

#include <stdint.h>

#define H9_PRIORITY_BIT_LENGTH 1
#define H9_TYPE_BIT_LENGTH 8
#define H9_RESERVED_BIT_LENGTH 2
#define H9_DESTINATION_ID_BIT_LENGTH 9
#define H9_SOURCE_ID_BIT_LENGTH 9

#define H9_RESERVED_VALUE 1

#define H9_BROADCAST_ID 0x1ff

#define H9_PRIORITY_HIGH 0
#define H9_PRIORITY_LOW 1

#define H9_TYPE_GROUP_MASK 0xC0

/* 00...... ALL RESERVED, for exampe for bootloader*/
#define H9_TYPE_GROUP_0 0

#define H9_TYPE_NOP 0
#define H9_TYPE_ENTER_INTO_BOOTLOADER 1
#define H9_TYPE_PAGE_START 2
#define H9_TYPE_PAGE_FILL_NEXT 3
#define H9_TYPE_PAGE_FILL 4
#define H9_TYPE_PAGE_WRITED 5
#define H9_TYPE_PAGE_FILL_BREAK 6
#define H9_TYPE_QUIT_BOOTLOADER 7
#define H9_TYPE_U8 8
#define H9_TYPE_U9 9
#define H9_TYPE_U10 10
#define H9_TYPE_U11 11
#define H9_TYPE_U12 12
#define H9_TYPE_U13 13
#define H9_TYPE_U14 14
#define H9_TYPE_U15 15
#define H9_TYPE_U16 16
#define H9_TYPE_U17 17
#define H9_TYPE_U18 18
#define H9_TYPE_U19 19
#define H9_TYPE_U20 20
#define H9_TYPE_U21 21
#define H9_TYPE_U22 22
#define H9_TYPE_U23 23
#define H9_TYPE_U24 24
#define H9_TYPE_U25 25
#define H9_TYPE_U26 26
#define H9_TYPE_U27 27
#define H9_TYPE_U28 28
#define H9_TYPE_U29 29
#define H9_TYPE_U30 30
#define H9_TYPE_U31 31
#define H9_TYPE_U32 32
#define H9_TYPE_U33 33
#define H9_TYPE_U34 34
#define H9_TYPE_U35 35
#define H9_TYPE_U36 36
#define H9_TYPE_U37 37
#define H9_TYPE_U38 38
#define H9_TYPE_U39 39
#define H9_TYPE_U40 40
#define H9_TYPE_U41 41
#define H9_TYPE_U42 42
#define H9_TYPE_U43 43
#define H9_TYPE_U44 44
#define H9_TYPE_U45 45
#define H9_TYPE_U46 46
#define H9_TYPE_U47 47
#define H9_TYPE_U48 48
#define H9_TYPE_U49 49
#define H9_TYPE_U50 50
#define H9_TYPE_U51 51
#define H9_TYPE_U52 52
#define H9_TYPE_U53 53
#define H9_TYPE_U54 54
#define H9_TYPE_U55 55
#define H9_TYPE_U56 56
#define H9_TYPE_U57 57
#define H9_TYPE_U58 58
#define H9_TYPE_U59 59
#define H9_TYPE_U60 60
#define H9_TYPE_U61 61
#define H9_TYPE_U62 62
#define H9_TYPE_U63 63

/* 01...... */
#define H9_TYPE_GROUP_1 64

#define H9_TYPE_REG_EXTERNALLY_CHANGED 64
#define H9_TYPE_REG_INTERNALLY_CHANGED 65
#define H9_TYPE_REG_VALUE_BROADCAST 66
#define H9_TYPE_REG_VALUE 67
#define H9_TYPE_NODE_HEARTBEAT 68
#define H9_TYPE_NODE_ERROR 69
#define H9_TYPE_U70 70
#define H9_TYPE_U71 71
#define H9_TYPE_U72 72
#define H9_TYPE_U73 73
#define H9_TYPE_U74 74
#define H9_TYPE_U75 75
#define H9_TYPE_U76 76
#define H9_TYPE_U77 77
#define H9_TYPE_U78 78
#define H9_TYPE_U79 79
#define H9_TYPE_U80 80
#define H9_TYPE_U81 81
#define H9_TYPE_U82 82
#define H9_TYPE_U83 83
#define H9_TYPE_U84 84
#define H9_TYPE_U85 85
#define H9_TYPE_U86 86
#define H9_TYPE_U87 87
#define H9_TYPE_U88 88
#define H9_TYPE_U89 89
#define H9_TYPE_U90 90
#define H9_TYPE_U91 91
#define H9_TYPE_U92 92
#define H9_TYPE_U93 93
#define H9_TYPE_U94 94
#define H9_TYPE_U95 95
#define H9_TYPE_U96 96
#define H9_TYPE_U97 97
#define H9_TYPE_U98 98
#define H9_TYPE_U99 99
#define H9_TYPE_U100 100
#define H9_TYPE_U101 101
#define H9_TYPE_U100 100
#define H9_TYPE_U101 101
#define H9_TYPE_U104 104
#define H9_TYPE_U105 105
#define H9_TYPE_U106 106
#define H9_TYPE_U107 107
#define H9_TYPE_U108 108
#define H9_TYPE_U109 109
#define H9_TYPE_U110 110
#define H9_TYPE_U111 111
#define H9_TYPE_U112 112
#define H9_TYPE_U113 113
#define H9_TYPE_U114 114
#define H9_TYPE_U115 115
#define H9_TYPE_U116 116
#define H9_TYPE_U117 117
#define H9_TYPE_U118 118
#define H9_TYPE_U119 119
#define H9_TYPE_U120 120
#define H9_TYPE_U121 121
#define H9_TYPE_U122 122
#define H9_TYPE_U123 123
#define H9_TYPE_U124 124
#define H9_TYPE_U125 125
#define H9_TYPE_U126 126
#define H9_TYPE_U127 127

/* 10...... */
#define H9_TYPE_GROUP_2 128

#define H9_TYPE_SET_REG 128
#define H9_TYPE_GET_REG 129
#define H9_TYPE_NODE_INFO 130
#define H9_TYPE_NODE_RESET 131
#define H9_TYPE_NODE_UPGRADE 132
#define H9_TYPE_U133 133
#define H9_TYPE_U134 134
#define H9_TYPE_U135 135
#define H9_TYPE_U136 136
#define H9_TYPE_U137 137
#define H9_TYPE_U138 138
#define H9_TYPE_U139 139
#define H9_TYPE_U140 140
#define H9_TYPE_U141 141
#define H9_TYPE_U142 142
#define H9_TYPE_U143 143
#define H9_TYPE_U144 144
#define H9_TYPE_U145 145
#define H9_TYPE_U146 146
#define H9_TYPE_U147 147
#define H9_TYPE_U148 148
#define H9_TYPE_U149 149
#define H9_TYPE_U150 150
#define H9_TYPE_U151 151
#define H9_TYPE_U152 152
#define H9_TYPE_U153 153
#define H9_TYPE_U154 154
#define H9_TYPE_U155 155
#define H9_TYPE_U156 156
#define H9_TYPE_U157 157
#define H9_TYPE_U158 158
#define H9_TYPE_U159 159
#define H9_TYPE_U160 160
#define H9_TYPE_U161 161
#define H9_TYPE_U162 162
#define H9_TYPE_U163 163
#define H9_TYPE_U164 164
#define H9_TYPE_U165 165
#define H9_TYPE_U166 166
#define H9_TYPE_U167 167
#define H9_TYPE_U168 168
#define H9_TYPE_U169 169
#define H9_TYPE_U170 170
#define H9_TYPE_U171 171
#define H9_TYPE_U172 172
#define H9_TYPE_U173 173
#define H9_TYPE_U174 174
#define H9_TYPE_U175 175
#define H9_TYPE_U176 176
#define H9_TYPE_U177 177
#define H9_TYPE_U178 178
#define H9_TYPE_U179 179
#define H9_TYPE_U180 180
#define H9_TYPE_U181 181
#define H9_TYPE_U182 182
#define H9_TYPE_U183 183
#define H9_TYPE_U184 184
#define H9_TYPE_U185 185
#define H9_TYPE_U186 186
#define H9_TYPE_U187 187
#define H9_TYPE_U188 188
#define H9_TYPE_U189 189
#define H9_TYPE_U190 190
#define H9_TYPE_U191 191

/* 11...... compatibility with broadcast */
#define H9_TYPE_GROUP_3 192

#define H9_TYPE_DISCOVERY 192
#define H9_TYPE_NODE_TURNED_ON 193
#define H9_TYPE_POWER_OFF 194
#define H9_TYPE_U195 195
#define H9_TYPE_U196 196
#define H9_TYPE_U197 197
#define H9_TYPE_U198 198
#define H9_TYPE_U199 199
#define H9_TYPE_U200 200
#define H9_TYPE_U201 201
#define H9_TYPE_U202 202
#define H9_TYPE_U203 203
#define H9_TYPE_U204 204
#define H9_TYPE_U205 205
#define H9_TYPE_U206 206
#define H9_TYPE_U207 207
#define H9_TYPE_U208 208
#define H9_TYPE_U209 209
#define H9_TYPE_U210 210
#define H9_TYPE_U211 211
#define H9_TYPE_U212 212
#define H9_TYPE_U213 213
#define H9_TYPE_U214 214
#define H9_TYPE_U215 215
#define H9_TYPE_U216 216
#define H9_TYPE_U217 217
#define H9_TYPE_U218 218
#define H9_TYPE_U219 219
#define H9_TYPE_U220 220
#define H9_TYPE_U221 221
#define H9_TYPE_U222 222
#define H9_TYPE_U223 223
#define H9_TYPE_U224 224
#define H9_TYPE_U225 225
#define H9_TYPE_U226 226
#define H9_TYPE_U227 227
#define H9_TYPE_U228 228
#define H9_TYPE_U229 229
#define H9_TYPE_U230 230
#define H9_TYPE_U231 231
#define H9_TYPE_U232 232
#define H9_TYPE_U233 233
#define H9_TYPE_U234 234
#define H9_TYPE_U235 235
#define H9_TYPE_U236 236
#define H9_TYPE_U237 237
#define H9_TYPE_U238 238
#define H9_TYPE_U239 239
#define H9_TYPE_U240 240
#define H9_TYPE_U241 241
#define H9_TYPE_U242 242
#define H9_TYPE_U243 243
#define H9_TYPE_U244 244
#define H9_TYPE_U245 245
#define H9_TYPE_U246 246
#define H9_TYPE_U247 247
#define H9_TYPE_U248 248
#define H9_TYPE_U249 249
#define H9_TYPE_U250 250
#define H9_TYPE_U251 251
#define H9_TYPE_U252 252
#define H9_TYPE_U253 253
#define H9_TYPE_U254 254
#define H9_TYPE_U255 255

// 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
// -- -- -- pp ty ty ty ty ty ty ty ty  0  1 ds ds ds ds ds ds ds ds ds so so so so so so so so so

struct h9msg {
	uint8_t priority :1;
	uint8_t type;
	uint16_t destination_id :9;
	uint16_t source_id :9;
	uint8_t dlc;
	uint8_t data[8];
};

typedef struct h9msg h9msg_t;

#endif /* H9MSG_H */
