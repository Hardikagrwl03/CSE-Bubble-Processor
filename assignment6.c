#include <stdio.h>


void insertion_sort(float arr[], int p)
{
    for (int i = 1; i < p; i++) {
        float temp = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > temp) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = temp;
    }
}

void bucket_sort(float arr[], int p, int n){
    
    float bucket[n][p]; 
    int bucket_size[n];
    for (int i = 0; i < n; i++) {
        bucket_size[i] = 0;
    }
    
    
    for(int i=0; i<p; i++){
        int bucket_index=n*arr[i];
        bucket[bucket_index][bucket_size[bucket_index]]=arr[i];
        bucket_size[bucket_index]++;
    }
    
    for (int i = 0; i < n; i++) {
        insertion_sort(bucket[i], bucket_size[i]);
    }

    int in=0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < bucket_size[i]; j++) {
            arr[in] = bucket[i][j];
            in++;
        }
    }
}


int main()
{ int p,n;
 scanf("%d %d",&p, &n);
 float arr[p];

   for(int i=0; i<p; i++){
       scanf("%f",&arr[i]);
   }
  
    bucket_sort(arr,p,n);
    
    for(int i=0; i<p; i++){
        printf("%0.7f ",arr[i]);
    }
  printf("\n");
    return 0;
}