## hdlc
HDLC framing

## Help
./hdlc_test --help
## Execute random HDLC packet encoding and decoding 
./hdlc_test

## HDLC encode the hex byte stream "12 7E 7E 34 56 78"
./hdlc_test -s "12 7E 7E 34 56 78"

## HDLC decode the hex byte stream "7E 12 7D 5E 7D 5E 34 56 78 02 A0 7E"
./hdlc_test -S "7E 12 7D 5E 7D 5E 34 56 78 02 A0 7E"
