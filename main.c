#include <stdio.h>

int IsBitSet(int b, int pos){
	return ((b >> pos) & 1) != 0;
}

int main() {
	int bits = 0b1000;
	if(IsBitSet(bits, 3)){
		printf("YES");
	}else{
		printf("NO");
	}
}
